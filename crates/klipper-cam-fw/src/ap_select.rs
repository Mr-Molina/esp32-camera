//! Multi-pass Wi-Fi access-point aggregation and roaming decisions.
//!
//! Pure, heap-free, host-testable logic. The firmware runs several active
//! scan passes, feeds every `(bssid, channel, rssi)` sample into an
//! [`ApTracker`], and picks the AP with the strongest *average* RSSI instead
//! of trusting a single sample. While associated, a [`WeakLinkTimer`] tracks
//! how long the link has stayed below [`ROAM_TRIGGER_RSSI_DBM`];
//! [`roam_decision`] then applies [`ROAM_HYSTERESIS_DB`] before switching.

#![forbid(unsafe_code)]

/// Number of active scan passes aggregated per AP selection (passes).
pub const SCAN_PASSES: u8 = 3;

/// Minimum active-scan dwell time per channel, in milliseconds.
pub const SCAN_ACTIVE_MIN_MS: u64 = 60;

/// Maximum active-scan dwell time per channel, in milliseconds.
pub const SCAN_ACTIVE_MAX_MS: u64 = 120;

/// RSSI below which the current link counts as weak, in dBm.
pub const ROAM_TRIGGER_RSSI_DBM: i8 = -65;

/// Continuous weak-link duration that triggers a roam scan, in milliseconds.
pub const ROAM_WEAK_DURATION_MS: u64 = 30_000;

/// Interval between link-quality (RSSI) checks, in milliseconds.
pub const ROAM_CHECK_INTERVAL_MS: u64 = 5_000;

/// Minimum spacing between two roam scans, in milliseconds.
pub const ROAM_MIN_SCAN_INTERVAL_MS: u64 = 120_000;

/// Upper bound of the roam scan backoff interval, in milliseconds.
///
/// Each roam scan that finds no better AP doubles the spacing to the next
/// scan, starting at [`ROAM_MIN_SCAN_INTERVAL_MS`], up to this cap
/// (15 minutes). See [`RoamBackoff`].
pub const ROAM_MAX_SCAN_INTERVAL_MS: u64 = 900_000;

/// Required RSSI advantage of a new AP over the current one, in dB.
pub const ROAM_HYSTERESIS_DB: i8 = 8;

/// RSSI penalty per missed scan pass relative to the highest hit count, in dB.
pub const MISSED_PASS_PENALTY_DB: i8 = 3;

/// Maximum number of distinct BSSIDs tracked at once (entries).
pub const MAX_AP_CANDIDATES: usize = 16;

/// Aggregated scan statistics for a single BSSID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ApCandidate {
    bssid: [u8; 6],
    channel: u8,
    rssi_sum: i32,
    rssi_max: i8,
    hits: u8,
}

impl ApCandidate {
    /// Placeholder used to initialise unused tracker slots.
    const EMPTY: Self = Self {
        bssid: [0; 6],
        channel: 0,
        rssi_sum: 0,
        rssi_max: i8::MIN,
        hits: 0,
    };

    /// Creates a candidate from its first sample.
    const fn first_sample(bssid: [u8; 6], channel: u8, rssi: i8) -> Self {
        Self {
            bssid,
            channel,
            rssi_sum: rssi as i32,
            rssi_max: rssi,
            hits: 1,
        }
    }

    /// Folds one more sample into the aggregate.
    fn add_sample(&mut self, channel: u8, rssi: i8) {
        self.channel = channel;
        if rssi > self.rssi_max {
            self.rssi_max = rssi;
        }
        // Once `hits` saturates, freeze the sum so the mean stays exact.
        if self.hits < u8::MAX {
            self.hits = self.hits.saturating_add(1);
            self.rssi_sum = self.rssi_sum.saturating_add(i32::from(rssi));
        }
    }

    /// Ranking key: effective RSSI, then average RSSI, then hit count, then
    /// peak RSSI.
    #[must_use]
    fn rank(&self, max_hits: u8) -> (i8, i8, u8, i8) {
        (
            self.effective_rssi(max_hits),
            self.rssi_avg(),
            self.hits,
            self.rssi_max,
        )
    }

    /// Effective RSSI in dBm after applying [`MISSED_PASS_PENALTY_DB`] for
    /// each missed pass relative to `max_hits`.
    ///
    /// Returns `i8::MIN` for a candidate without samples.
    #[must_use]
    pub fn effective_rssi(&self, max_hits: u8) -> i8 {
        if self.hits == 0 {
            return i8::MIN;
        }
        let missed = i16::from(max_hits.saturating_sub(self.hits));
        let penalty = i16::from(MISSED_PASS_PENALTY_DB).saturating_mul(missed);
        let eff = i16::from(self.rssi_avg())
            .saturating_sub(penalty)
            .clamp(i16::from(i8::MIN), i16::from(i8::MAX));
        i8::try_from(eff).unwrap_or(i8::MIN)
    }

    /// BSSID (AP MAC address).
    #[must_use]
    pub const fn bssid(&self) -> [u8; 6] {
        self.bssid
    }

    /// Primary channel of the most recent sample.
    #[must_use]
    pub const fn channel(&self) -> u8 {
        self.channel
    }

    /// Mean RSSI in dBm, rounded half away from zero.
    ///
    /// Returns `i8::MIN` for a candidate without samples.
    #[must_use]
    pub fn rssi_avg(&self) -> i8 {
        let hits = i32::from(self.hits);
        if hits == 0 {
            return i8::MIN;
        }
        let bias = if self.rssi_sum < 0 { -hits } else { hits };
        let doubled = self.rssi_sum.saturating_mul(2).saturating_add(bias);
        let avg = doubled
            .checked_div(hits.saturating_mul(2))
            .unwrap_or(i32::from(i8::MIN));
        i8::try_from(avg).unwrap_or(if avg < 0 { i8::MIN } else { i8::MAX })
    }

    /// Strongest single RSSI sample, in dBm.
    #[must_use]
    pub const fn rssi_max(&self) -> i8 {
        self.rssi_max
    }

    /// Number of scan samples aggregated (saturates at `u8::MAX`).
    #[must_use]
    pub const fn hits(&self) -> u8 {
        self.hits
    }
}

/// Fixed-capacity aggregator of scan results across multiple passes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ApTracker {
    entries: [ApCandidate; MAX_AP_CANDIDATES],
    len: usize,
}

impl Default for ApTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl ApTracker {
    /// Creates an empty tracker.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: [ApCandidate::EMPTY; MAX_AP_CANDIDATES],
            len: 0,
        }
    }

    /// Records one scan sample.
    ///
    /// Updates the entry for `bssid` if present, otherwise inserts it. When
    /// the table is full the weakest entry is replaced only if `rssi` is
    /// stronger than that entry's average.
    pub fn record(&mut self, bssid: [u8; 6], channel: u8, rssi: i8) {
        let Some(active) = self.entries.get_mut(..self.len) else {
            return;
        };
        for entry in active.iter_mut() {
            if entry.bssid == bssid {
                entry.add_sample(channel, rssi);
                return;
            }
        }
        let fresh = ApCandidate::first_sample(bssid, channel, rssi);
        if let Some(slot) = self.entries.get_mut(self.len) {
            *slot = fresh;
            self.len = self.len.saturating_add(1);
            return;
        }
        let Some(weakest_idx) = self.weakest_index() else {
            return;
        };
        let Some(weakest) = self.entries.get_mut(weakest_idx) else {
            return;
        };
        if rssi > weakest.rssi_avg() {
            *weakest = fresh;
        }
    }

    /// Highest hit count among all recorded candidates (`0` when empty).
    #[must_use]
    pub fn max_hits(&self) -> u8 {
        let mut max = 0_u8;
        for entry in self.candidates() {
            if entry.hits > max {
                max = entry.hits;
            }
        }
        max
    }

    /// Index of the lowest-ranked active entry.
    #[must_use]
    fn weakest_index(&self) -> Option<usize> {
        let max_hits = self.max_hits();
        let mut weakest: Option<(usize, (i8, i8, u8, i8))> = None;
        for (idx, entry) in self.candidates().iter().enumerate() {
            let rank = entry.rank(max_hits);
            let is_weaker = match weakest {
                Some((_, best_rank)) => rank < best_rank,
                None => true,
            };
            if is_weaker {
                weakest = Some((idx, rank));
            }
        }
        weakest.map(|(idx, _)| idx)
    }

    /// Strongest candidate: highest effective RSSI, ties broken by average
    /// RSSI, then by more hits, then by higher peak RSSI.
    #[must_use]
    pub fn best(&self) -> Option<ApCandidate> {
        self.best_filtered(None)
    }

    /// Strongest candidate whose BSSID differs from `bssid`.
    #[must_use]
    pub fn best_excluding(&self, bssid: [u8; 6]) -> Option<ApCandidate> {
        self.best_filtered(Some(bssid))
    }

    /// Shared ranking loop for [`Self::best`] and [`Self::best_excluding`].
    #[must_use]
    fn best_filtered(&self, excluded: Option<[u8; 6]>) -> Option<ApCandidate> {
        let max_hits = self.max_hits();
        let mut best: Option<ApCandidate> = None;
        for entry in self.candidates() {
            if excluded == Some(entry.bssid) {
                continue;
            }
            let is_better = match best {
                Some(current) => entry.rank(max_hits) > current.rank(max_hits),
                None => true,
            };
            if is_better {
                best = Some(*entry);
            }
        }
        best
    }

    /// All recorded candidates, in insertion order.
    #[must_use]
    pub fn candidates(&self) -> &[ApCandidate] {
        self.entries.get(..self.len).unwrap_or(&[])
    }

    /// Number of recorded candidates.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns `true` when no candidate has been recorded.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Forgets all recorded candidates.
    pub fn clear(&mut self) {
        self.entries = [ApCandidate::EMPTY; MAX_AP_CANDIDATES];
        self.len = 0;
    }
}

/// Measures how long the link RSSI has continuously stayed below a threshold.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WeakLinkTimer {
    weak_since_ms: Option<u64>,
}

impl WeakLinkTimer {
    /// Creates a timer in the "link strong" state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            weak_since_ms: None,
        }
    }

    /// Feeds an RSSI sample taken at `now_ms` (monotonic milliseconds).
    ///
    /// Returns the continuous weak duration in milliseconds: `0` when
    /// `rssi >= threshold_dbm` (which also resets the timer) or on the first
    /// weak sample.
    pub fn update(&mut self, now_ms: u64, rssi: i8, threshold_dbm: i8) -> u64 {
        if rssi >= threshold_dbm {
            self.weak_since_ms = None;
            return 0;
        }
        let since = *self.weak_since_ms.get_or_insert(now_ms);
        now_ms.saturating_sub(since)
    }

    /// Returns the timer to the "link strong" state.
    pub fn reset(&mut self) {
        self.weak_since_ms = None;
    }
}

/// Exponential backoff for the spacing between roam scans.
///
/// Starts at [`ROAM_MIN_SCAN_INTERVAL_MS`], doubles after every scan that
/// finds no better AP, and saturates at [`ROAM_MAX_SCAN_INTERVAL_MS`]. This
/// bounds the main-loop stall caused by scans when the best reachable AP
/// stays below [`ROAM_TRIGGER_RSSI_DBM`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoamBackoff {
    interval_ms: u64,
}

impl RoamBackoff {
    /// Creates a backoff at the minimum interval
    /// ([`ROAM_MIN_SCAN_INTERVAL_MS`]).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            interval_ms: ROAM_MIN_SCAN_INTERVAL_MS,
        }
    }

    /// Returns the required spacing before the next roam scan, in
    /// milliseconds.
    #[must_use]
    pub const fn interval_ms(&self) -> u64 {
        self.interval_ms
    }

    /// Doubles the interval after a scan found no better AP, capped at
    /// [`ROAM_MAX_SCAN_INTERVAL_MS`].
    pub fn on_no_better_ap(&mut self) {
        self.interval_ms = self
            .interval_ms
            .saturating_mul(2)
            .min(ROAM_MAX_SCAN_INTERVAL_MS);
    }

    /// Returns the interval to [`ROAM_MIN_SCAN_INTERVAL_MS`], e.g. after a
    /// roam attempt or a fresh (re)association.
    pub fn reset(&mut self) {
        self.interval_ms = ROAM_MIN_SCAN_INTERVAL_MS;
    }
}

impl Default for RoamBackoff {
    fn default() -> Self {
        Self::new()
    }
}

/// Outcome of [`roam_decision`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RoamDecision {
    /// Keep the current association.
    Stay,
    /// Re-associate with the contained candidate.
    Roam(ApCandidate),
}

/// Decides whether to roam away from `current_bssid`.
///
/// Roams only if the best *other* BSSID has an effective RSSI of at least
/// `current_rssi + hysteresis_db` dBm. The sum is computed in `i16` with
/// saturating arithmetic, so the boundary stays exact at the `i8` edges
/// (e.g. `127 + 8` is never clamped back to `127`).
#[must_use]
pub fn roam_decision(
    current_bssid: [u8; 6],
    current_rssi: i8,
    tracker: &ApTracker,
    hysteresis_db: i8,
) -> RoamDecision {
    let required = i16::from(current_rssi).saturating_add(i16::from(hysteresis_db));
    let max_hits = tracker.max_hits();
    match tracker.best_excluding(current_bssid) {
        Some(candidate) if i16::from(candidate.effective_rssi(max_hits)) >= required => {
            RoamDecision::Roam(candidate)
        }
        _ => RoamDecision::Stay,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const AP_A: [u8; 6] = [0xAA, 0, 0, 0, 0, 1];
    const AP_B: [u8; 6] = [0xBB, 0, 0, 0, 0, 2];
    const AP_C: [u8; 6] = [0xCC, 0, 0, 0, 0, 3];

    fn bssid_n(n: u8) -> [u8; 6] {
        [0x10, 0, 0, 0, 0, n]
    }

    #[test]
    fn tunables_match_spec() {
        assert_eq!(SCAN_PASSES, 3);
        assert_eq!(SCAN_ACTIVE_MIN_MS, 60);
        assert_eq!(SCAN_ACTIVE_MAX_MS, 120);
        assert_eq!(ROAM_TRIGGER_RSSI_DBM, -65);
        assert_eq!(ROAM_WEAK_DURATION_MS, 30_000);
        assert_eq!(ROAM_CHECK_INTERVAL_MS, 5_000);
        assert_eq!(ROAM_MIN_SCAN_INTERVAL_MS, 120_000);
        assert_eq!(ROAM_HYSTERESIS_DB, 8);
        assert_eq!(MAX_AP_CANDIDATES, 16);
    }

    #[test]
    fn empty_tracker_has_no_best() {
        let tracker = ApTracker::new();
        assert!(tracker.is_empty());
        assert_eq!(tracker.best(), None);
        assert_eq!(tracker.best_excluding(AP_A), None);
        assert!(tracker.candidates().is_empty());
    }

    #[test]
    fn three_pass_aggregation_missed_pass_still_wins() {
        let mut tracker = ApTracker::new();
        // Pass 1
        tracker.record(AP_A, 1, -70);
        tracker.record(AP_B, 6, -60);
        // Pass 2: B missed, A has a lucky spike above B's average.
        tracker.record(AP_A, 1, -58);
        // Pass 3
        tracker.record(AP_A, 1, -72);
        tracker.record(AP_B, 6, -62);

        let a = tracker.candidates()[0];
        assert_eq!(a.hits(), 3);
        assert_eq!(a.rssi_avg(), -67);
        assert_eq!(a.rssi_max(), -58);

        let best = tracker.best().expect("tracker has entries");
        assert_eq!(best.bssid(), AP_B);
        assert_eq!(best.channel(), 6);
        assert_eq!(best.hits(), 2);
        assert_eq!(best.rssi_avg(), -61);
        assert_eq!(tracker.len(), 2);
    }

    #[test]
    fn flaky_single_hit_loses_to_consistent_three_hit_candidate() {
        let mut tracker = ApTracker::new();
        // Flaky candidate: 1/3 hit at -63 dBm (avg = -63, eff = -69)
        tracker.record(AP_A, 1, -63);
        // Consistent candidate: 3/3 hits at -62, -66, -67 dBm (avg = -65, eff = -65)
        tracker.record(AP_B, 6, -62);
        tracker.record(AP_B, 6, -66);
        tracker.record(AP_B, 6, -67);

        assert_eq!(tracker.max_hits(), 3);
        let flaky = tracker.candidates()[0];
        let consistent = tracker.candidates()[1];
        assert_eq!(flaky.hits(), 1);
        assert_eq!(flaky.rssi_avg(), -63);
        assert_eq!(flaky.effective_rssi(tracker.max_hits()), -69);
        assert_eq!(consistent.hits(), 3);
        assert_eq!(consistent.rssi_avg(), -65);
        assert_eq!(consistent.effective_rssi(tracker.max_hits()), -65);

        let best = tracker.best().expect("tracker has entries");
        assert_eq!(best.bssid(), AP_B);
    }

    #[test]
    fn rssi_avg_rounds_half_away_from_zero() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, -60);
        tracker.record(AP_A, 1, -61);
        assert_eq!(tracker.candidates()[0].rssi_avg(), -61);

        tracker.record(AP_B, 1, -60);
        tracker.record(AP_B, 1, -60);
        tracker.record(AP_B, 1, -61);
        // -181 / 3 = -60.33 -> -60
        assert_eq!(tracker.candidates()[1].rssi_avg(), -60);
    }

    #[test]
    fn channel_tracks_latest_sample() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, -60);
        tracker.record(AP_A, 11, -60);
        assert_eq!(tracker.candidates()[0].channel(), 11);
    }

    #[test]
    fn tie_break_prefers_more_hits() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, -60);
        tracker.record(AP_B, 6, -60);
        tracker.record(AP_B, 6, -60);
        assert_eq!(tracker.best().map(|c| c.bssid()), Some(AP_B));
    }

    #[test]
    fn tie_break_then_prefers_higher_max() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, -60);
        tracker.record(AP_A, 1, -60);
        tracker.record(AP_B, 6, -55);
        tracker.record(AP_B, 6, -65);
        assert_eq!(tracker.best().map(|c| c.bssid()), Some(AP_B));
    }

    #[test]
    fn full_table_replaces_weakest_only_when_stronger() {
        let mut tracker = ApTracker::new();
        for i in 0..16_u8 {
            tracker.record(bssid_n(i), 1, -50 - i as i8);
        }
        assert_eq!(tracker.len(), MAX_AP_CANDIDATES);

        // Weaker than the weakest (-65): ignored.
        tracker.record(AP_A, 3, -66);
        assert_eq!(tracker.len(), MAX_AP_CANDIDATES);
        assert!(tracker.candidates().iter().all(|c| c.bssid() != AP_A));

        // Equal to the weakest: ignored.
        tracker.record(AP_A, 3, -65);
        assert!(tracker.candidates().iter().all(|c| c.bssid() != AP_A));

        // Stronger than the weakest: replaces bssid_n(15).
        tracker.record(AP_A, 3, -60);
        assert_eq!(tracker.len(), MAX_AP_CANDIDATES);
        assert!(tracker.candidates().iter().any(|c| c.bssid() == AP_A));
        assert!(tracker
            .candidates()
            .iter()
            .all(|c| c.bssid() != bssid_n(15)));

        // Existing BSSIDs still update in place when full.
        tracker.record(bssid_n(0), 1, -50);
        let first = tracker.candidates()[0];
        assert_eq!(first.bssid(), bssid_n(0));
        assert_eq!(first.hits(), 2);
    }

    #[test]
    fn best_excluding_skips_current_bssid() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, -40);
        tracker.record(AP_B, 6, -70);
        tracker.record(AP_C, 11, -60);
        assert_eq!(tracker.best().map(|c| c.bssid()), Some(AP_A));
        assert_eq!(tracker.best_excluding(AP_A).map(|c| c.bssid()), Some(AP_C));
        assert_eq!(tracker.best_excluding(AP_B).map(|c| c.bssid()), Some(AP_A));

        let mut single = ApTracker::new();
        single.record(AP_A, 1, -40);
        assert_eq!(single.best_excluding(AP_A), None);
    }

    #[test]
    fn clear_empties_tracker() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, -40);
        tracker.clear();
        assert!(tracker.is_empty());
        assert_eq!(tracker.best(), None);
        assert_eq!(tracker, ApTracker::default());
    }

    #[test]
    fn weak_timer_accumulates_and_resets() {
        let mut timer = WeakLinkTimer::new();
        assert_eq!(timer.update(0, -70, ROAM_TRIGGER_RSSI_DBM), 0);
        assert_eq!(timer.update(10_000, -70, ROAM_TRIGGER_RSSI_DBM), 10_000);
        assert_eq!(timer.update(30_000, -80, ROAM_TRIGGER_RSSI_DBM), 30_000);
        // Exactly at threshold counts as strong and resets.
        assert_eq!(timer.update(31_000, -65, ROAM_TRIGGER_RSSI_DBM), 0);
        assert_eq!(timer.update(32_000, -70, ROAM_TRIGGER_RSSI_DBM), 0);
        assert_eq!(timer.update(39_000, -70, ROAM_TRIGGER_RSSI_DBM), 7_000);
        timer.reset();
        assert_eq!(timer.update(40_000, -70, ROAM_TRIGGER_RSSI_DBM), 0);
        assert_eq!(timer.update(45_000, -70, ROAM_TRIGGER_RSSI_DBM), 5_000);
    }

    #[test]
    fn weak_timer_clock_going_backwards_saturates() {
        let mut timer = WeakLinkTimer::new();
        assert_eq!(timer.update(5_000, -90, -65), 0);
        assert_eq!(timer.update(1_000, -90, -65), 0);
    }

    #[test]
    fn hysteresis_boundary_exact_plus_8_roams_plus_7_stays() {
        let mut roam = ApTracker::new();
        roam.record(AP_A, 1, -70);
        roam.record(AP_B, 6, -62);
        assert_eq!(
            roam_decision(AP_A, -70, &roam, ROAM_HYSTERESIS_DB),
            RoamDecision::Roam(roam.candidates()[1])
        );

        let mut stay = ApTracker::new();
        stay.record(AP_A, 1, -70);
        stay.record(AP_B, 6, -63);
        assert_eq!(
            roam_decision(AP_A, -70, &stay, ROAM_HYSTERESIS_DB),
            RoamDecision::Stay
        );
    }

    #[test]
    fn roam_never_targets_current_bssid() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, -30);
        assert_eq!(
            roam_decision(AP_A, -90, &tracker, ROAM_HYSTERESIS_DB),
            RoamDecision::Stay
        );
        assert_eq!(
            roam_decision(AP_A, -90, &ApTracker::new(), ROAM_HYSTERESIS_DB),
            RoamDecision::Stay
        );
    }

    #[test]
    fn i8_saturation_edges() {
        let mut tracker = ApTracker::new();
        tracker.record(AP_A, 1, i8::MIN);
        tracker.record(AP_A, 1, i8::MIN);
        tracker.record(AP_B, 1, i8::MAX);
        tracker.record(AP_B, 1, i8::MAX);
        assert_eq!(tracker.candidates()[0].rssi_avg(), i8::MIN);
        assert_eq!(tracker.candidates()[1].rssi_avg(), i8::MAX);

        // 127 + 8 must not clamp to 127: an equal-strength AP never wins.
        assert_eq!(
            roam_decision(AP_A, i8::MAX, &tracker, 8),
            RoamDecision::Stay
        );
        // Extreme hysteresis on top of the max stays.
        assert_eq!(
            roam_decision(AP_A, i8::MAX, &tracker, i8::MAX),
            RoamDecision::Stay
        );
        // From the floor, +8 -> -120 boundary.
        let mut floor = ApTracker::new();
        floor.record(AP_A, 1, i8::MIN);
        floor.record(AP_B, 1, -120);
        assert!(matches!(
            roam_decision(AP_A, i8::MIN, &floor, 8),
            RoamDecision::Roam(_)
        ));
        let mut below = ApTracker::new();
        below.record(AP_B, 1, -121);
        assert_eq!(roam_decision(AP_A, i8::MIN, &below, 8), RoamDecision::Stay);
        // Most negative hysteresis from the floor roams to anything.
        assert!(matches!(
            roam_decision(AP_A, i8::MIN, &floor, i8::MIN),
            RoamDecision::Roam(_)
        ));
    }

    #[test]
    fn mixed_sign_average_and_hit_saturation() {
        let mut tracker = ApTracker::new();
        for _ in 0..300 {
            tracker.record(AP_A, 1, -60);
        }
        let a = tracker.candidates()[0];
        assert_eq!(a.hits(), u8::MAX);
        assert_eq!(a.rssi_avg(), -60);

        tracker.record(AP_B, 1, 3);
        tracker.record(AP_B, 1, 2);
        // 2.5 -> 3
        assert_eq!(tracker.candidates()[1].rssi_avg(), 3);
    }

    #[test]
    fn roam_backoff_doubles_to_cap_and_resets() {
        assert_eq!(ROAM_MAX_SCAN_INTERVAL_MS, 900_000);
        let mut backoff = RoamBackoff::new();
        assert_eq!(backoff, RoamBackoff::default());
        assert_eq!(backoff.interval_ms(), 120_000);
        backoff.on_no_better_ap();
        assert_eq!(backoff.interval_ms(), 240_000);
        backoff.on_no_better_ap();
        assert_eq!(backoff.interval_ms(), 480_000);
        backoff.on_no_better_ap();
        assert_eq!(backoff.interval_ms(), 900_000);
        for _ in 0..100 {
            backoff.on_no_better_ap();
        }
        assert_eq!(backoff.interval_ms(), ROAM_MAX_SCAN_INTERVAL_MS);
        backoff.reset();
        assert_eq!(backoff.interval_ms(), ROAM_MIN_SCAN_INTERVAL_MS);
    }
}

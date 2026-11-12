#!/bin/bash
# Claude Code cloud environment setup script.
#
# Cloud sessions ignore `extraKnownMarketplaces` / `enabledPlugins` in the
# repo's .claude/settings.json (no workspace trust dialog in the cloud), so
# plugins must be installed here, before Claude Code launches.
#
# Paste this file's contents into the cloud environment's "Setup script"
# field on claude.ai/code. Every step is non-fatal so a plugin outage never
# blocks a session from starting.

# Microsoft TokenEconomizer
claude plugin marketplace add microsoft/TokenEconomizer || true
claude plugin install tokeneconomizer@tokeneconomizer -y || true

# Superpowers: cloud sessions already get the official copy synced from the
# claude.ai account. Uncomment to use the Mr-Molina fork instead, and turn the
# synced copy off on claude.ai so the two don't load side by side.
# claude plugin marketplace add Mr-Molina/superpowers || true
# claude plugin install superpowers@superpowers-dev -y || true

exit 0

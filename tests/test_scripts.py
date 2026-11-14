"""Unit tests for repository automation and verification scripts."""

import hashlib
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

# ``scripts/`` is placed on sys.path by tests/conftest.py before this module
# loads, which is what lets these sit with the other imports.
import ensure_serial_passive
import ota_deploy
import verify_cold_boot_invariants

REPO_ROOT = Path(__file__).resolve().parent.parent


class TestOtaDeploy(unittest.TestCase):
    """Tests for ota_deploy.py functions and error handling."""

    def test_expected_next_slot_valid(self):
        self.assertEqual(ota_deploy.expected_next_slot("ota_0"), "ota_1")
        self.assertEqual(ota_deploy.expected_next_slot("ota_1"), "ota_0")

    def test_expected_next_slot_invalid(self):
        invalid_slots = ["", "factory", "ota_2", "invalid_slot", "OTA_0"]
        for slot in invalid_slots:
            with self.subTest(slot=slot):
                with self.assertRaises(ValueError) as ctx:
                    ota_deploy.expected_next_slot(slot)
                self.assertIn("Unknown active_partition label", str(ctx.exception))

    def test_missing_host_exit_1(self):
        with (
            patch.object(sys, "argv", ["ota_deploy.py"]),
            patch.dict(os.environ, {"CAM_HOST": "", "CAM_OTA_TOKEN": ""}),
            patch("sys.stderr"),
        ):
            exit_code = ota_deploy.main()
            self.assertEqual(exit_code, 1)

    def test_read_http_response_content_length(self):
        body = b"Firmware update successful! Rebooting...\n"
        headers = (
            b"HTTP/1.1 200 OK\r\n"
            b"Content-Type: text/plain\r\n"
            b"Content-Length: " + str(len(body)).encode("utf-8") + b"\r\n"
            b"Connection: close\r\n\r\n"
        )
        full_packet = headers + body

        mock_sock = MagicMock()
        mock_sock.recv.side_effect = [full_packet[:30], full_packet[30:], b""]

        resp_text = ota_deploy.read_http_response(mock_sock)
        self.assertIn("200 OK", resp_text)
        self.assertIn("Firmware update successful! Rebooting...", resp_text)

    def test_read_http_response_invalid_raises_runtime_error(self):
        mock_sock = MagicMock()
        mock_sock.recv.side_effect = [
            b"HTTP/1.1 500 Internal Server Error\r\nContent-Length: 0\r\n\r\n",
            b"",
        ]

        with self.assertRaises(RuntimeError) as ctx:
            ota_deploy.read_http_response(mock_sock)
        self.assertIn("Unexpected HTTP response", str(ctx.exception))

    def test_app_sha256_check(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp_path = Path(tmpdir)
            elf_file = tmp_path / "dummy.elf"
            bin_file = tmp_path / "dummy.bin"
            elf_file.write_bytes(b"\x7fELF" + b"\x00" * 100)

            # Build valid 24-byte ESP image header:
            # magic=0xE9, segment_count=1, spi_mode=0, spi_speed=0, entry=0, hash_appended=1 (byte 23)
            header = bytearray(24)
            header[0] = 0xE9
            header[1] = 1
            header[23] = 1  # hash_appended = True

            payload_body = header + b"ESP32_FIRMWARE_PAYLOAD_TEST_DATA"
            valid_digest = hashlib.sha256(payload_body).digest()

            # Case A: Corrupt appended SHA-256
            corrupt_payload = payload_body + b"\x00" * 32
            bin_file.write_bytes(corrupt_payload)

            with (
                patch("shutil.which", return_value="/bin/espflash"),
                patch("os.path.isfile", side_effect=lambda p: True),
                patch("subprocess.run") as mock_run,
            ):
                mock_run.return_value = MagicMock(returncode=0, stdout="", stderr="")
                with self.assertRaises(ValueError) as ctx:
                    ota_deploy.stage_and_validate_image(
                        str(elf_file), str(bin_file), host="127.0.0.1", port=80
                    )
                self.assertIn("Appended SHA-256 mismatch", str(ctx.exception))

            # Case B: Valid appended SHA-256
            valid_payload = payload_body + valid_digest
            bin_file.write_bytes(valid_payload)
            with (
                patch("shutil.which", return_value="/bin/espflash"),
                patch("os.path.isfile", side_effect=lambda p: True),
                patch("subprocess.run") as mock_run,
            ):
                mock_run.return_value = MagicMock(returncode=0, stdout="", stderr="")
                res_bytes = ota_deploy.stage_and_validate_image(
                    str(elf_file), str(bin_file), host="127.0.0.1", port=80
                )
                self.assertEqual(res_bytes, valid_payload)


class TestEnsureSerialPassive(unittest.TestCase):
    """Tests for ensure_serial_passive.py CLI and environment handling."""

    def test_cli_and_env_defaults(self):
        with patch.dict(
            os.environ, {"CAM_SERIAL_PORT": "/dev/ttyUSB-test", "CAM_HOST": "192.0.2.1"}
        ):
            import importlib

            importlib.reload(ensure_serial_passive)

            self.assertEqual(
                ensure_serial_passive.DEFAULT_SERIAL_PORT, "/dev/ttyUSB-test"
            )
            self.assertEqual(ensure_serial_passive.DEFAULT_HOST, "192.0.2.1")
            self.assertEqual(
                ensure_serial_passive.DEFAULT_STATUS_URL, "http://192.0.2.1/status"
            )

        # Reload with clean env
        with patch.dict(os.environ, {}, clear=True):
            import importlib

            importlib.reload(ensure_serial_passive)
            self.assertEqual(
                ensure_serial_passive.DEFAULT_SERIAL_PORT, "/dev/ttyUSB-esp32"
            )
            self.assertEqual(ensure_serial_passive.DEFAULT_HOST, "")
            self.assertEqual(ensure_serial_passive.DEFAULT_STATUS_URL, "")

    def test_non_linux_platform_guard(self):
        if sys.platform != "linux":
            # On non-linux (e.g. Windows), ensure_passive returns 1 safely
            with patch("sys.stderr"):
                ret = ensure_serial_passive.ensure_passive(check_http=False)
                self.assertEqual(ret, 1)


class TestVerifyColdBootInvariants(unittest.TestCase):
    """Tests for verify_cold_boot_invariants.py static and binary checks."""

    def test_missing_bootloader_exit_1_without_flag(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            nonexistent = Path(tmpdir) / "missing_bootloader.bin"
            errors = verify_cold_boot_invariants.verify_bootloader_bin(
                allow_missing=False, bootloader_path=nonexistent
            )
            self.assertTrue(len(errors) > 0)
            self.assertIn("Missing patched bootloader binary", errors[0])

    def test_missing_bootloader_exit_0_with_allow_missing(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            nonexistent = Path(tmpdir) / "missing_bootloader.bin"
            errors = verify_cold_boot_invariants.verify_bootloader_bin(
                allow_missing=True, bootloader_path=nonexistent
            )
            self.assertEqual(errors, [])

    def test_corrupt_bootloader_fails_verification(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            corrupt_bin = Path(tmpdir) / "corrupt_bootloader.bin"

            # Case A: Too small or bad magic byte
            corrupt_bin.write_bytes(b"\x00" * 1024)
            errors = verify_cold_boot_invariants.verify_bootloader_bin(
                allow_missing=False, bootloader_path=corrupt_bin
            )
            self.assertTrue(
                any("Invalid ESP32 bootloader header or size" in e for e in errors)
            )

            # Case B: Correct magic and size, but missing MMIO register and 0x2B00 sequence
            valid_header_empty_body = bytearray(16384)
            valid_header_empty_body[0] = 0xE9
            corrupt_bin.write_bytes(bytes(valid_header_empty_body))
            errors = verify_cold_boot_invariants.verify_bootloader_bin(
                allow_missing=False, bootloader_path=corrupt_bin
            )
            self.assertTrue(any("missing IO_MUX_GPIO16_REG" in e for e in errors))
            self.assertTrue(
                any("missing GPIO 16 pull-up constant 0x2B00" in e for e in errors)
            )

    def test_full_script_allow_missing_bootloader_main(self):
        with patch.object(
            sys,
            "argv",
            ["verify_cold_boot_invariants.py", "--allow-missing-bootloader"],
        ):
            ret = verify_cold_boot_invariants.main()
            self.assertEqual(ret, 0)


if __name__ == "__main__":
    unittest.main()

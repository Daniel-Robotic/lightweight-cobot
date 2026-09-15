"""Update recovery checks; external git and installer operations are isolated."""

import io
import unittest
from unittest.mock import patch

from rich.console import Console

from cobot.commands import update


class UpdateTests(unittest.TestCase):
    def run_update(self, codes, behind="1"):
        output = io.StringIO()
        commands = []

        def stream(command, **kwargs):
            commands.append(command)
            result = next(codes, 0)
            if isinstance(result, Exception):
                raise result
            return result

        def git(*args):
            return {"rev-parse": "controller", "rev-list": behind,
                    "log": "abc123 Update CLI"}[args[0]]

        console = Console(file=output, color_system=None)
        with patch.object(update, "_git", side_effect=git), \
                patch.object(update.process, "stream", side_effect=stream), \
                patch("cobot.process.console", console), \
                patch("cobot.ui.console", console):
            code = update._update()
        return code, output.getvalue(), commands

    def test_install_failure_reports_failure(self):
        for install_code in (1, -9, -15):
            with self.subTest(install_code=install_code):
                code, output, _ = self.run_update(iter([0, 0, install_code]))
                self.assertEqual(code, 1)
                self.assertNotIn("Проект обновлён", output)
                self.assertIn("cobot update", output)

    def test_current_checkout_still_retries_installation(self):
        code, _, commands = self.run_update(iter([0, 0]), behind="0")
        self.assertEqual([command[:2] for command in commands],
                         [["git", "fetch"], ["uv", "tool"]])
        self.assertEqual(code, 0)

    def test_killed_git_stops_before_next_step(self):
        for codes, expected_count in [([-9], 1), ([0, -15], 2)]:
            with self.subTest(codes=codes):
                code, output, commands = self.run_update(iter(codes))
                self.assertNotEqual(code, 0)
                self.assertEqual(len(commands), expected_count)
                self.assertNotIn("Проект обновлён", output)

    def test_missing_installer_reports_failure_without_traceback(self):
        code, output, _ = self.run_update(
            iter([0, 0, FileNotFoundError("uv not found")]))
        self.assertNotEqual(code, 0)
        self.assertIn("uv not found", output)

    def test_command_exits_nonzero_after_failed_installation(self):
        with patch.object(update, "_update", return_value=1):
            with self.assertRaises(SystemExit) as result:
                update.run(None)
        self.assertEqual(result.exception.code, 1)


if __name__ == "__main__":
    unittest.main()

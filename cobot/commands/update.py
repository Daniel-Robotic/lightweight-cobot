from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

from cobot import process
from cobot.ui import done
from cobot.process import StepProgress

_PROJECT_DIR = Path(__file__).parent.parent.parent


def _git(*args: str) -> str:
    """Run a git command in the project dir and return its stripped stdout.
    Запускает git-команду в директории проекта и возвращает обрезанный stdout.
    """
    return subprocess.check_output(["git", *args], cwd=_PROJECT_DIR, text=True).strip()


def _update() -> int:
    """Fetch the current branch, show incoming commits, pull, and reinstall the cobot CLI.
    Progress: fetch (0-30 %), pull (30-80 %), reinstall (80-100 %).
    Получает текущую ветку, показывает входящие коммиты, делает pull и переустанавливает CLI.
    Прогресс: fetch (0-30 %), pull (30-80 %), переустановка (80-100 %).
    """
    ok, fail_msg = True, ""
    with StepProgress("Обновление проекта") as p:
        try:
            branch = _git("rev-parse", "--abbrev-ref", "HEAD")
            p.raw(f"[cyan]▸[/cyan] Ветка: [bold]{branch}[/bold]")

            p.set(0, "Получение с удалённого репозитория...")
            rc = process.stream(["git", "fetch", "origin"], cwd=str(_PROJECT_DIR), on_line=p.log)
            if rc != 0:
                done(False, f"git fetch завершился с кодом {rc}")
                return 1
            p.set(30)

            behind = _git("rev-list", f"HEAD..origin/{branch}", "--count")
            if behind == "0":
                p.log("Исходники уже актуальны; проверка установки CLI...")
            else:
                p.raw(f"\n[bold]{behind} новых коммит(ов):[/bold]")
                for line in _git("log", f"HEAD..origin/{branch}", "--oneline").splitlines():
                    p.log(line)

                p.set(30, "Применение изменений...")
                rc = process.stream(["git", "pull", "origin", branch],
                                   cwd=str(_PROJECT_DIR), on_line=p.log)
                if rc != 0:
                    done(False, f"git pull завершился с кодом {rc}")
                    return 1
            p.set(80)

            p.set(80, "Переустановка cobot CLI...")
            p.raw("\n[cyan]▸[/cyan] Переустановка cobot CLI...")
            rc = process.stream(["uv", "tool", "install", "--editable", str(_PROJECT_DIR)],
                               on_line=p.log)
            if rc == 0:
                p.raw("[green]✓[/green] cobot переустановлен")
                p.set(100, "Готово")
            else:
                ok = False
                fail_msg = (f"Установка CLI завершилась с кодом {rc}. "
                            "Исходники обновлены, но установка не завершена. "
                            "Сохраните вывод ошибки выше и повторите cobot update.")
        except (subprocess.CalledProcessError, OSError) as exc:
            ok, fail_msg = False, str(exc)

    done(ok, "Проект обновлён" if ok else fail_msg)
    return 0 if ok else 1


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("update", help="Pull latest changes from the remote git branch")
    p.set_defaults(func=run)


def run(args: argparse.Namespace) -> None:
    raise SystemExit(_update())

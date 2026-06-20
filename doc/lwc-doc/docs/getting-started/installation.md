# Установка проекта

## Быстрая установка

Самый простой способ — установить через `curl` одной командой. Убедитесь, что `curl` установлен:

```bash
sudo apt update && sudo apt upgrade -y && sudo apt install curl
```

Перейдите в домашнюю директорию и запустите скрипт установки:

=== "Стабильная версия (master)"

    ```bash
    cd ~
    curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/master/install.sh | bash
    ```

=== "Dev-версия (dev)"

    ```bash
    cd ~
    curl -fsSL https://gitverse.ru/api/repos/daniel-robotics/lightweight-cobot/raw/branch/dev/install.sh | bash -s dev
    ```

---

## Установка через Git

Проект доступен как на [GitVerse](https://gitverse.ru/daniel-robotics/lightweight-cobot) (предпочтительно), так и на [GitHub](https://github.com/Daniel-Robotic/lightweight-cobot).

!!! tip "Не знакомы с Git?"
    Если вы впервые работаете с Git и GitHub, рекомендуем [прочитать эту статью на Хабре](https://habr.com/ru/companies/yandex_praktikum/articles/700708/) — там всё объясняется с нуля.

Клонируйте репозиторий и запустите скрипт установки:

=== "GitVerse"

    ```bash
    cd ~
    git clone https://gitverse.ru/daniel-robotics/lightweight-cobot.git
    cd ~/lightweight-cobot
    sudo chmod +x ./install.sh
    ./install.sh
    ```

=== "GitHub"

    ```bash
    cd ~
    git clone https://github.com/Daniel-Robotic/lightweight-cobot.git
    cd ~/lightweight-cobot
    sudo chmod +x ./install.sh
    ./install.sh
    ```

---

## Ручная установка

Если ни `curl`, ни `git` недоступны — скачайте архив проекта вручную со страницы репозитория (кнопка «Скачать ZIP»), разархивируйте и выполните:

```bash
cd ~/lightweight-cobot
sudo chmod +x ./install.sh
./install.sh
```

---

## Процесс установки

Скрипт [`install.sh`](https://gitverse.ru/daniel-robotics/lightweight-cobot/raw/branch/master/install.sh) автоматически установит:

- **Git** — система контроля версий
- **Docker** — контейнеризация для изолированного запуска
- **CLI-инструмент `cobot`** — основной инструмент управления проектом
- Системные зависимости Ubuntu

### Перезагрузка после установки

После завершения скрипта может потребоваться перезагрузка, чтобы заработал Docker:

```bash
sudo reboot now
```

Следите за выводом в терминале — скрипт сам сообщит, нужна ли перезагрузка.

### Если лог пуст

Если после запуска скрипта не выводится никакой информации, обновите среду bash и продолжите настройку вручную:

```bash
source ~/.bashrc   # обновление переменных окружения
cobot setup        # продолжение настройки системы
```

---

## Проверка установки

После завершения убедитесь, что `cobot` доступен:

```bash
cobot -h
```

Если команда выводит список доступных подкоманд — установка прошла успешно.

---

**Следующий шаг:** [CLI-команды cobot](cli-reference.md)

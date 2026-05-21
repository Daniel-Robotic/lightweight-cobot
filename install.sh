#!/bin/bash

# Stop the script immediately if any command exits with an error.
# Останавливаем скрипт сразу, если какая-либо команда завершилась с ошибкой.
set -e

# Disable uv user/project config files so the environment is always clean.
# Отключаем пользовательские и проектные конфиги uv, чтобы среда всегда была чистой.
export UV_NO_CONFIG=1

# Terminal color codes for nicer output.
# Коды цветов для красивого вывода в терминал.
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

# Python version that the cobot CLI requires.
# Версия Python, которая нужна для работы cobot CLI.
PYTHON_VERSION="3.11"
REPO_URL="https://gitverse.ru/daniel-robotics/lightweight-cobot.git"

# Where to clone the project. Can be overridden by the user with COBOT_INSTALL_DIR.
# Куда клонировать проект. Пользователь может переопределить через COBOT_INSTALL_DIR.
INSTALL_DIR="${COBOT_INSTALL_DIR:-$HOME/.lwc}"

# Detect interactive mode - when run via curl | bash, stdin is not a terminal.
# Определяем интерактивный режим - при запуске через curl | bash stdin не является терминалом.
if [ -t 0 ]; then IS_INTERACTIVE=true; else IS_INTERACTIVE=false; fi

# Logging helpers - one line per severity level.
# Вспомогательные функции логирования - одна строка на уровень важности.
log_info()    { echo -e "${CYAN}[*]${NC} $1"; }
log_success() { echo -e "${GREEN}[ok]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[!]${NC} $1"; }
log_error()   { echo -e "${RED}[err]${NC} $1"; exit 1; }

# Run a command silently and only print its output if it fails.
# This makes the normal install look clean while still showing errors when something breaks.
# Запускает команду тихо и показывает вывод только если она завершилась с ошибкой.
# Это делает обычную установку аккуратной, но при ошибке мы всё равно видим детали.
run_quiet() {
    local _log
    _log="$(mktemp /tmp/lwc-cmd.XXXXXX.log)"
    if "$@" </dev/null >"$_log" 2>&1; then
        rm -f "$_log"
    else
        cat "$_log" >&2
        rm -f "$_log"
        return 1
    fi
}

print_banner() {
    echo ""
    echo -e "${BOLD}"
    echo "┌─────────────────────────────────────────────────────────┐"
    echo "│               Lightweight Cobot installer               │"
    echo "└─────────────────────────────────────────────────────────┘"
    echo -e "${NC}"
}

# Detect the current OS and package manager so later steps know how to install things.
# Определяем текущую ОС и пакетный менеджер, чтобы следующие шаги знали как устанавливать пакеты.
detect_os() {
    case "$(uname -s)" in
        Linux*)
            OS="linux"
            # Pick the first package manager we can find on this system.
            # Выбираем первый найденный пакетный менеджер.
            if command -v apt-get &>/dev/null; then PKG_MANAGER="apt"
            elif command -v dnf &>/dev/null; then PKG_MANAGER="dnf"
            elif command -v pacman &>/dev/null; then PKG_MANAGER="pacman"
            else PKG_MANAGER="unknown"
            fi
            ;;
        Darwin*) OS="macos" ;;
        *) log_error "Unsupported OS: $(uname -s)" ;;
    esac
    log_info "OS: $OS"
}

# Install system packages using whatever package manager was detected above.
# Устанавливает системные пакеты через найденный пакетный менеджер.
pkg_install() {
    case "$PKG_MANAGER" in
        apt)    run_quiet sudo apt-get update -qq && run_quiet sudo apt-get install -y --no-install-recommends "$@" ;;
        dnf)    run_quiet sudo dnf install -y "$@" ;;
        pacman) run_quiet sudo pacman -S --noconfirm "$@" ;;
        *)      log_error "Cannot auto-install $* — unknown package manager" ;;
    esac
}

# Check if git is installed and install it if not.
# Проверяем наличие git и устанавливаем его если он отсутствует.
check_git() {
    log_info "Checking git..."
    if command -v git &>/dev/null; then
        log_success "git $(git --version | awk '{print $3}') found"
        return
    fi
    log_info "Installing git..."
    case "$OS" in
        linux) pkg_install git ;;
        macos) run_quiet xcode-select --install 2>/dev/null || run_quiet brew install git ;;
    esac
    command -v git &>/dev/null || log_error "Failed to install git"
    log_success "git $(git --version | awk '{print $3}') installed"
}

# Check if Docker is installed and install it if not.
# Проверяем наличие Docker и устанавливаем его если он отсутствует.
check_docker() {
    log_info "Checking Docker..."
    if command -v docker &>/dev/null; then
        log_success "Docker $(docker --version | awk '{print $3}' | tr -d ',') found"
        return
    fi
    log_info "Installing Docker..."
    # Download the installer to a temp file instead of piping directly through bash.
    # This way network errors and installer errors are shown separately.
    # Скачиваем установщик во временный файл, а не запускаем через pipe.
    # Так ошибки сети и ошибки самого установщика видны по отдельности.
    local _installer
    _installer="$(mktemp /tmp/lwc-docker.XXXXXX.sh)"
    if ! curl -fsSL https://get.docker.com -o "$_installer"; then
        rm -f "$_installer"
        log_error "Failed to download Docker installer"
    fi
    run_quiet sh "$_installer" || { rm -f "$_installer"; log_error "Docker installation failed"; }
    rm -f "$_installer"
    command -v docker &>/dev/null || log_error "Docker not found after installation"
    log_success "Docker $(docker --version | awk '{print $3}' | tr -d ',') installed"
    # Add the current user to the docker group so sudo is not needed every time.
    # Добавляем текущего пользователя в группу docker, чтобы не требовался sudo каждый раз.
    if [ "$(id -u)" -ne 0 ] && command -v usermod &>/dev/null; then
        sudo usermod -aG docker "$USER"
        log_warn "Added $USER to docker group — re-login to apply"
    fi
}

# Install the uv package manager. We need it to create isolated Python environments.
# Устанавливаем пакетный менеджер uv. Он нужен для создания изолированных Python-окружений.
install_uv() {
    log_info "Checking uv..."
    # uv can end up in ~/.local/bin or ~/.cargo/bin depending on how it was installed.
    # uv может оказаться в ~/.local/bin или ~/.cargo/bin в зависимости от способа установки.
    UV_CMD=""
    for candidate in "uv" "$HOME/.local/bin/uv" "$HOME/.cargo/bin/uv"; do
        if command -v "$candidate" &>/dev/null 2>&1; then
            UV_CMD="$candidate"
            break
        fi
    done
    if [ -n "$UV_CMD" ]; then
        log_success "uv $($UV_CMD --version | awk '{print $2}') found"
        return
    fi
    log_info "Installing uv..."
    # Two separate temp files - one for the installer script and one for its log.
    # This lets us tell apart "download failed" from "installer failed".
    # Два отдельных временных файла - для установщика и для его лога.
    # Это позволяет различить ошибку скачивания и ошибку самого установщика.
    local _log _installer
    _log="$(mktemp /tmp/lwc-uv.XXXXXX.log)"
    _installer="$(mktemp /tmp/lwc-uv-installer.XXXXXX.sh)"
    if ! curl -LsSf https://astral.sh/uv/install.sh -o "$_installer" 2>"$_log"; then
        cat "$_log" >&2; rm -f "$_log" "$_installer"
        log_error "Failed to download uv installer"
    fi
    if sh "$_installer" >> "$_log" 2>&1; then
        rm -f "$_installer"
        for candidate in "$HOME/.local/bin/uv" "$HOME/.cargo/bin/uv"; do
            [ -x "$candidate" ] && UV_CMD="$candidate" && break
        done
        if [ -z "$UV_CMD" ]; then
            cat "$_log" >&2; rm -f "$_log"
            log_error "uv installed but binary not found"
        fi
        rm -f "$_log"
        log_success "uv $($UV_CMD --version | awk '{print $2}') installed"
    else
        cat "$_log" >&2; rm -f "$_log" "$_installer"
        log_error "Failed to install uv"
    fi
}

# Check that the required Python version is available via uv and install it if not.
# Проверяем наличие нужной версии Python через uv и устанавливаем её если она отсутствует.
check_python() {
    log_info "Checking Python $PYTHON_VERSION..."
    local py_path
    py_path="$("$UV_CMD" python find "$PYTHON_VERSION" </dev/null 2>/dev/null)" || true
    if [ -n "$py_path" ]; then
        local ver
        ver="$("$py_path" --version 2>&1)" || true
        log_success "${ver:-Python $PYTHON_VERSION} found"
        return
    fi
    log_info "Installing Python $PYTHON_VERSION via uv..."
    run_quiet "$UV_CMD" python install "$PYTHON_VERSION" || log_error "Failed to install Python $PYTHON_VERSION"
    log_success "Python $PYTHON_VERSION installed"
}

# Figure out where the project lives. Three cases are handled:
#   1. We are already inside the cloned repo - use it directly.
#   2. The repo was cloned before - just pull the latest changes.
#   3. First time - clone the repo fresh.
# Определяем где находится проект. Обрабатываем три случая:
#   1. Мы уже внутри клонированного репозитория - используем его напрямую.
#   2. Репозиторий уже был клонирован ранее - просто тянем последние изменения.
#   3. Первый запуск - клонируем репозиторий заново.
resolve_install_dir() {
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" 2>/dev/null && pwd || pwd)"

    # Running directly from inside the cloned repo (not via curl|bash)
    if [ -f "$script_dir/setup.py" ] && [ -d "$script_dir/.git" ]; then
        INSTALL_DIR="$script_dir"
        log_info "Using existing repo: $INSTALL_DIR"
        return
    fi

    # Repo already cloned — pull instead of re-clone
    if [ -d "$INSTALL_DIR/.git" ]; then
        log_info "Repo already exists at $INSTALL_DIR — pulling latest..."
        git -C "$INSTALL_DIR" pull --ff-only origin dev </dev/null \
            && log_success "Repo updated" \
            || log_warn "Could not pull latest — using existing version"
        return
    fi

    # Directory exists but is not a git repo (broken/partial) — remove it
    if [ -d "$INSTALL_DIR" ]; then
        log_warn "Removing incomplete directory $INSTALL_DIR..."
        rm -rf "$INSTALL_DIR"
    fi

    log_info "Cloning repo into $INSTALL_DIR..."
    mkdir -p "$(dirname "$INSTALL_DIR")"
    # Retry up to 5 times because the git server can be unreliable on slow connections.
    # Повторяем до 5 раз, потому что git-сервер может быть нестабильным на медленных соединениях.
    local attempt=1
    while [ $attempt -le 5 ]; do
        # TODO: Изменить на --depth 1 --branch main после слияния dev в main.
        if git clone --depth 1 --branch dev "$REPO_URL" "$INSTALL_DIR" </dev/null; then
            log_success "Repo cloned to $INSTALL_DIR"
            return
        fi
        log_warn "Clone failed (attempt $attempt/5), retrying..."
        rm -rf "$INSTALL_DIR"
        attempt=$((attempt + 1))
    done
    log_error "Failed to clone repo after 5 attempts"
}

# Install the cobot CLI as an editable uv tool so changes in the source are reflected immediately.
# Устанавливаем cobot CLI как редактируемый uv-инструмент, чтобы изменения в исходниках применялись сразу.
install_cobot() {
    log_info "Installing cobot CLI..."
    cd "$INSTALL_DIR"
    # uv tool install creates an isolated environment and puts the cobot binary into ~/.local/bin.
    # uv tool install создаёт изолированное окружение и кладёт бинарник cobot в ~/.local/bin.
    run_quiet "$UV_CMD" tool install --python "$PYTHON_VERSION" --editable . \
        || log_error "Failed to install cobot"
    log_success "cobot installed"
}

# Add ~/.local/bin to PATH in the user's shell config if it is not there yet.
# Добавляем ~/.local/bin в PATH в конфиге оболочки пользователя, если его там ещё нет.
setup_path() {
    local bin_dir="$HOME/.local/bin"
    local shell_rc
    case "$SHELL" in
        */zsh)  shell_rc="$HOME/.zshrc" ;;
        */fish) shell_rc="$HOME/.config/fish/config.fish" ;;
        *)      shell_rc="$HOME/.bashrc" ;;
    esac
    # Also export into the current session right now so cobot setup works below without a re-login.
    # Также экспортируем прямо сейчас, чтобы cobot setup заработал ниже без перезахода.
    if [[ ":$PATH:" != *":$bin_dir:"* ]]; then
        echo "" >> "$shell_rc"
        echo "export PATH=\"$bin_dir:\$PATH\"" >> "$shell_rc"
        export PATH="$bin_dir:$PATH"
    fi
    command -v cobot &>/dev/null && log_success "cobot -> $(command -v cobot)"
}

print_success() {
    echo ""
    echo -e "${GREEN}${BOLD}Installation complete!${NC}"
    echo ""
    echo "  Next steps:"
    echo ""
    echo -e "  ${CYAN}cobot setup${NC}         — install ROS2, build the project and configure the robot"
    echo -e "  ${CYAN}cobot --help${NC}        — show all available commands"
    echo ""
}

# Launch the interactive setup wizard right after installation.
# Запускаем интерактивный мастер настройки сразу после установки.
run_setup() {
    # Explicit check because PATH might not include ~/.local/bin yet in this shell session.
    # Явная проверка, потому что PATH может ещё не включать ~/.local/bin в этой сессии.
    if ! command -v cobot &>/dev/null; then
        log_warn "cobot not found on PATH, trying full path..."
        local cobot_bin="$HOME/.local/bin/cobot"
        if [ -x "$cobot_bin" ]; then
            export PATH="$HOME/.local/bin:$PATH"
        else
            log_error "cobot binary not found — try: source ~/.bashrc && cobot setup"
        fi
    fi

    log_info "Running cobot setup..."
    cobot setup

    # When run via curl | bash the child process cannot update the parent terminal's environment.
    # При запуске через curl | bash дочерний процесс не может обновить окружение родительского терминала.
    if [ "$IS_INTERACTIVE" = false ]; then
        echo ""
        echo "  To apply PATH changes in this terminal, run:"
        echo "    source ~/.bashrc   (bash)"
        echo "    source ~/.zshrc    (zsh)"
        echo ""
    fi
}

main() {
    print_banner
    detect_os
    check_git
    check_docker
    install_uv
    check_python
    resolve_install_dir
    install_cobot
    setup_path
    print_success
    run_setup
}

main

#!/bin/bash

set -e

export UV_NO_CONFIG=1

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

PYTHON_VERSION="3.11"
REPO_URL="https://gitverse.ru/daniel-robotics/lightweight-cobot.git"
INSTALL_DIR="${COBOT_INSTALL_DIR:-$HOME/.lwc/ros2_iiwa7}"

# Определяем интерактивный режим: при запуске через curl | bash stdin не является терминалом
if [ -t 0 ]; then IS_INTERACTIVE=true; else IS_INTERACTIVE=false; fi

log_info()    { echo -e "${CYAN}[*]${NC} $1"; }
log_success() { echo -e "${GREEN}[ok]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[!]${NC} $1"; }
log_error()   { echo -e "${RED}[err]${NC} $1"; exit 1; }

# Запускает команду тихо, показывает вывод только при ошибке
run_quiet() {
    local _log
    _log="$(mktemp /tmp/lwc-cmd.XXXXXX.log)"
    if "$@" > "$_log" 2>&1; then
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
    echo "│             KUKA iiwa7 ROS2 Control Framework           │"
    echo "└─────────────────────────────────────────────────────────┘"
    echo -e "${NC}"
}

detect_os() {
    case "$(uname -s)" in
        Linux*)
            OS="linux"
            # Определяем пакетный менеджер для установки зависимостей
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

# Устанавливает системные пакеты через найденный пакетный менеджер
pkg_install() {
    case "$PKG_MANAGER" in
        apt)    run_quiet sudo apt-get update -qq && run_quiet sudo apt-get install -y --no-install-recommends "$@" ;;
        dnf)    run_quiet sudo dnf install -y "$@" ;;
        pacman) run_quiet sudo pacman -S --noconfirm "$@" ;;
        *)      log_error "Cannot auto-install $* — unknown package manager" ;;
    esac
}

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

check_docker() {
    log_info "Checking Docker..."
    if command -v docker &>/dev/null; then
        log_success "Docker $(docker --version | awk '{print $3}' | tr -d ',') found"
        return
    fi
    log_info "Installing Docker..."
    # Скачиваем установщик во временный файл, а не запускаем через pipe —
    # так видны ошибки сети отдельно от ошибок самого установщика
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
    # Добавляем пользователя в группу docker, чтобы не требовался sudo
    if [ "$(id -u)" -ne 0 ] && command -v usermod &>/dev/null; then
        sudo usermod -aG docker "$USER"
        log_warn "Added $USER to docker group — re-login to apply"
    fi
}

install_uv() {
    log_info "Checking uv..."
    # uv может быть установлен в ~/.local/bin или ~/.cargo/bin, проверяем оба
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
    # Два отдельных файла: лог и установщик — чтобы различать ошибки скачивания и установки
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

check_python() {
    log_info "Checking Python $PYTHON_VERSION..."
    if "$UV_CMD" python find "$PYTHON_VERSION" &>/dev/null; then
        local ver
        ver="$("$UV_CMD" python find "$PYTHON_VERSION" | xargs -I{} {} --version 2>&1)"
        log_success "$ver found"
        return
    fi
    # uv умеет скачивать и изолировать нужную версию Python без sudo
    log_info "Installing Python $PYTHON_VERSION via uv..."
    run_quiet "$UV_CMD" python install "$PYTHON_VERSION" || log_error "Failed to install Python $PYTHON_VERSION"
    log_success "Python $PYTHON_VERSION installed"
}

resolve_install_dir() {
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" 2>/dev/null && pwd || pwd)"
    # Если рядом со скриптом есть setup.py — значит мы уже внутри репозитория
    if [ -f "$script_dir/setup.py" ]; then
        INSTALL_DIR="$script_dir"
        log_info "Repo: $INSTALL_DIR"
    else
        # Иначе клонируем в ~/.lwc/ros2_iiwa7
        log_info "Cloning repo..."
        mkdir -p "$(dirname "$INSTALL_DIR")"
        # TODO: Убрать ветку dev и юзать main после мержа
        run_quiet git clone --branch dev "$REPO_URL" "$INSTALL_DIR" \
            || log_error "Failed to clone repo"
        log_success "Repo cloned to $INSTALL_DIR"
    fi
}

install_cobot() {
    log_info "Installing cobot CLI..."
    cd "$INSTALL_DIR"
    # uv tool install создаёт изолированное окружение и кладёт бинарник cobot в ~/.local/bin
    run_quiet "$UV_CMD" tool install --python "$PYTHON_VERSION" --editable . \
        || log_error "Failed to install cobot"
    log_success "cobot installed"
}

setup_path() {
    local bin_dir="$HOME/.local/bin"
    local shell_rc
    case "$SHELL" in
        */zsh)  shell_rc="$HOME/.zshrc" ;;
        */fish) shell_rc="$HOME/.config/fish/config.fish" ;;
        *)      shell_rc="$HOME/.bashrc" ;;
    esac
    # Добавляем ~/.local/bin в PATH, если его там ещё нет
    if [[ ":$PATH:" != *":$bin_dir:"* ]]; then
        echo "" >> "$shell_rc"
        echo "export PATH=\"$bin_dir:\$PATH\"" >> "$shell_rc"
        # Обновляем PATH внутри скрипта — нужно чтобы cobot setup сработал ниже
        export PATH="$bin_dir:$PATH"
    fi
    command -v cobot &>/dev/null && log_success "cobot -> $(command -v cobot)"
}

print_success() {
    echo ""
    echo -e "${GREEN}${BOLD}Done!${NC}"
    echo ""
    echo "  cobot setup    - first-time setup"
    echo "  cobot --help   - list all commands"
    echo ""
}

run_setup() {
    # Явная проверка — PATH мог не подхватиться если uv положил бинарник в нестандартное место
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

    # curl | bash: дочерний процесс не может обновить терминал родителя
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

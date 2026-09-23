#!/usr/bin/env bash
# One-time setup of the VIGIL master launcher. Changes nothing inside the three projects.
#   bash ~/Documents/vigil/setup_vigil.sh
#  1. makes the launcher executable
#  2. installs the `vigil` command (~/.local/bin/vigil -> ~/Documents/vigil/vigil)
#  3. creates SHORTCUT LINKS to the three projects (they stay where they are; nothing is moved or
#     copied, so their builds and absolute paths keep working):
#        agriculture/ros2_ws                 -> ~/Documents/robot/ros2_ws
#        rock_terrain/vigil_rough_terrain_ws -> ~/Documents/robot/vigil_rough_terrain_ws
#        military_sar/military_world         -> ~/Documents/military_world
set -eu
VIGIL="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
chmod +x "$VIGIL/vigil" "$VIGIL/master_launcher/vigil_launcher.py"
mkdir -p "$HOME/.local/bin"
ln -sfn "$VIGIL/vigil" "$HOME/.local/bin/vigil"
echo "installed: $HOME/.local/bin/vigil"
link() {  # link <shortcut> <real project folder>
  if [[ -d "$2" ]]; then
    mkdir -p "$(dirname "$VIGIL/$1")"
    if [[ -e "$VIGIL/$1" && ! -L "$VIGIL/$1" ]]; then echo "skipped $1 (a real folder is already there)"; return; fi
    ln -sfn "$2" "$VIGIL/$1"; echo "link:      $VIGIL/$1 -> $2"
  else
    echo "not found: $2 (no link for $1)"
  fi
}
link agriculture/ros2_ws                 "$HOME/Documents/robot/ros2_ws"
if [[ -d "$HOME/Documents/robot/vigil_rough_terrain_ws" ]]; then
  link rock_terrain/vigil_rough_terrain_ws "$HOME/Documents/robot/vigil_rough_terrain_ws"
else
  link rock_terrain/vigil_rough_terrain_ws "$HOME/Documents/robot/ros2_ws/vigil_rough_terrain_ws"
fi
link military_sar/military_world         "$HOME/Documents/military_world"
case ":$PATH:" in
  *":$HOME/.local/bin:"*) echo "ready: type  vigil" ;;
  *) echo 'add ~/.local/bin to PATH once:  echo '"'"'export PATH="$HOME/.local/bin:$PATH"'"'"' >> ~/.bashrc && source ~/.bashrc'
     echo "then type  vigil" ;;
esac

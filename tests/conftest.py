from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
for p in ['living_map_beacons','living_map_radio','living_map_gateway','living_map_executor',
          'living_map_writer','living_map_link_sim','living_map_command_post']:
 sys.path.insert(0,str(ROOT/'src'/p))

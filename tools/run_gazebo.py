#!/usr/bin/env python3
import os
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
args = ['gz', 'sim', '-r', '-v', '3']
if len(sys.argv) > 1 and sys.argv[1].lower() in ('false', '0', 'no'):
    args.append('-s')
args.append(str(root / 'worlds/test_world.sdf'))
os.execvp('gz', args)

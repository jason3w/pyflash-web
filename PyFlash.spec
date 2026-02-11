# -*- mode: python ; coding: utf-8 -*-


a = Analysis(
    ['auto_flash.py'],
    pathex=[],
    binaries=[],
    datas=[('Puya.PY32F0xx_DFP.1.2.8.pack', '.'), ('index.html', '.'), ('status.json', '.'), ('Glass.aiff', '.'), ('Ping.aiff', '.')],
    hiddenimports=[],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.datas,
    [],
    name='PyFlash',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
app = BUNDLE(
    exe,
    name='PyFlash.app',
    icon=None,
    bundle_identifier=None,
)

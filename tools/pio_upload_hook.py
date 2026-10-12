"""PlatformIO extra script: the Upload button runs tools/upload_auto.py.
It updates this firmware over USB, or does a full install over another firmware (see upload_auto.py)."""
Import("env")  # noqa: F821 (PlatformIO SCons)

images = " ".join(f'{off} "{env.subst(path)}"' for off, path in env.get("FLASH_EXTRA_IMAGES", []))  # noqa: F821
env.Replace(  # noqa: F821
    UPLOADCMD='"$PYTHONEXE" "$PROJECT_DIR/tools/upload_auto.py" --port "$UPLOAD_PORT" --esptool "$UPLOADER" '
              + images + ' $ESP32_APP_OFFSET "$SOURCE"')

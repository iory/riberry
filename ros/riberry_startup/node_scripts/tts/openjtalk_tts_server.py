#!/usr/bin/env python3

import argparse
import hashlib
import os
import shutil
import tempfile
import warnings

from fastapi import BackgroundTasks
from fastapi import FastAPI
from fastapi import HTTPException
from fastapi.responses import FileResponse
import numpy as np
from pydantic import BaseModel
import pyopenjtalk
from scipy.io import wavfile
import uvicorn

print("[Text2Wave Server] Initializing pyopenjtalk...")
pyopenjtalk.tts("初期化")  # Initialize with dummy text
print("[Text2Wave Server] Ready to serve requests")


def get_cache_dir():
    """Return cache dir.

    Returns
    -------
    cache_dir : str
        cache directory.
    """
    ros_home = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))
    pkg_ros_home = os.path.join(ros_home, 'openjtalk')
    default_cache_dir = os.path.join(pkg_ros_home, 'cache')
    cache_dir = os.environ.get(
        'ROS_OPENJTALK_CACHE_DIR',
        default_cache_dir)
    if not os.path.exists(cache_dir):
        os.makedirs(cache_dir)
    return cache_dir


def checksum_md5(filename, blocksize=8192):
    """Calculate md5sum.

    Parameters
    ----------
    filename : str or pathlib.Path
        input filename.
    blocksize : int
        MD5 has 128-byte digest blocks (default: 8192 is 128x64).
    Returns
    -------
    md5 : str
        calculated md5sum.
    """
    filename = str(filename)
    hash_factory = hashlib.md5()
    with open(filename, 'rb') as f:
        for chunk in iter(lambda: f.read(blocksize), b''):
            hash_factory.update(chunk)
    return hash_factory.hexdigest()


def checksum_md5_from_text(text, lang='en'):
    """Calculate md5sum from text and language."""
    content = f"{text}--{lang}".encode()
    return hashlib.md5(content).hexdigest()


def convert_to_str(x):
    if isinstance(x, str):
        pass
    elif isinstance(x, bytes):
        x = x.decode('utf-8')
    else:
        raise ValueError(
            f'Invalid input x type: {type(x)}'
            )
    return x


def request_synthesis(
        sentence, output_path, lang='en'):
    sentence = convert_to_str(sentence)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        x, sr = pyopenjtalk.tts(sentence)
        wavfile.write(str(output_path), sr, x.astype(np.int16))


# Initialize FastAPI app
app = FastAPI(title="Text2Wave Server", version="1.0.0")


class TTSRequest(BaseModel):
    text: str
    lang: str = 'ja'
    use_cache: bool = True


@app.get("/")
async def root():
    return {"message": "Text2Wave Server is running"}


@app.post("/synthesize")
async def synthesize_speech(request: TTSRequest, background_tasks: BackgroundTasks):
    """Synthesize speech from text."""
    try:
        # Validate language
        if request.lang not in ['ja', 'en']:
            request.lang = 'en'

        # Check cache if enabled
        if request.use_cache:
            cache_dir = get_cache_dir()
            md5 = checksum_md5_from_text(request.text, request.lang)
            cache_filename = os.path.join(
                cache_dir,
                f"{md5}--{request.lang}.wav"
            )

            if os.path.exists(cache_filename):
                print(f'[Text2Wave Server] Using cached sound file ({cache_filename}) for {request.text}')
                return FileResponse(
                    cache_filename,
                    media_type="audio/wav",
                    filename="speech.wav"
                )

        # Create temporary file for synthesis
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
            output_path = tmp_file.name

        # Synthesize speech
        request_synthesis(request.text, output_path, request.lang)

        # Save to cache if enabled
        if request.use_cache:
            text_cache_filename = os.path.splitext(cache_filename)[0] + '.txt'
            print(f'[Text2Wave Server] Cache saved to {cache_filename}')

            # Save text and wav to cache
            with open(text_cache_filename, 'w', encoding='utf-8') as f:
                f.write(request.text)
            shutil.copy(output_path, cache_filename)

        # Clean up temp file after response (only if not cached)
        if not request.use_cache:
            background_tasks.add_task(os.unlink, output_path)

        # Return the synthesized audio file
        return FileResponse(
            output_path,
            media_type="audio/wav",
            filename="speech.wav"
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


if __name__ == "__main__":
    # Filter out ROS arguments
    import sys
    ros_args = [arg for arg in sys.argv[1:] if arg.startswith('__')]
    other_args = [arg for arg in sys.argv[1:] if not arg.startswith('__')]

    parser = argparse.ArgumentParser(description='Text2Wave Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind to')
    parser.add_argument('--port', type=int, default=8000, help='Port to bind to')
    parser.add_argument('--reload', action='store_true', help='Enable auto-reload')
    args = parser.parse_args(other_args)

    uvicorn.run(
        "openjtalk_tts_server:app",
        host=args.host,
        port=args.port,
        reload=args.reload
    )

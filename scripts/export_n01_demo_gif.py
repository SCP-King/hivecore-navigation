#!/usr/bin/env python3
"""Extract an ACTUAL RViz recording into a short GIF and one PNG frame.

No AI-rendered/fabricated obstacle visualization. ffmpeg reads the user's MP4.
Both outputs remain review artifacts, not real-time safety evidence.
"""
import argparse
from pathlib import Path
import subprocess
import sys


def run(args):
    subprocess.run(args,check=True,stdout=subprocess.DEVNULL)


def export(video, output, still, start_s=18., length_s=12., width=960):
    video=Path(video)
    output=Path(output)
    still=Path(still)
    if not video.is_file() or video.stat().st_size<1024:
        raise ValueError("Missing/nontrivial REAL recording MP4")
    if not (0.<=start_s<=10000. and 1.<=length_s<=30.
            and 320<=width<=1600):
        raise ValueError("GIF preview bounds invalid")
    output.parent.mkdir(parents=True,exist_ok=True)
    still.parent.mkdir(parents=True,exist_ok=True)
    palette=output.with_suffix(".palette.png")
    common=["ffmpeg","-hide_banner","-loglevel","error","-nostdin","-y"]
    try:
        run(common+["-ss",str(start_s),"-t",str(length_s),"-i",str(video),
                    "-vf",f"fps=6,scale={width}:-1:flags=lanczos,palettegen",
                    str(palette)])
        run(common+["-ss",str(start_s),"-t",str(length_s),"-i",str(video),
                    "-i",str(palette),"-filter_complex",
                    f"[0:v]fps=6,scale={width}:-1:flags=lanczos[x];"
                    "[x][1:v]paletteuse=dither=bayer:bayer_scale=3",
                    str(output)])
        run(common+["-ss",str(start_s+1.),"-i",str(video),
                    "-frames:v","1","-update","1",str(still)])
    finally:
        palette.unlink(missing_ok=True)
    if not output.exists() or output.stat().st_size<1024:
        raise RuntimeError("No real GIF frames produced")
    if not still.exists() or still.stat().st_size<1024:
        raise RuntimeError("No real screenshot frame produced")
    return output,still


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video",required=True)
    parser.add_argument("--gif",required=True)
    parser.add_argument("--still",required=True)
    parser.add_argument("--start",type=float,default=18.)
    parser.add_argument("--seconds",type=float,default=12.)
    args=parser.parse_args()
    try:
        gif,still=export(args.video,args.gif,args.still,
                         start_s=args.start,length_s=args.seconds)
    except (ValueError,RuntimeError,subprocess.CalledProcessError,
            FileNotFoundError) as exc:
        print(f"Gif export FAILED: {exc}",file=sys.stderr)
        return 1
    print(f"Actual RViz recording GIF: {gif}")
    print(f"Actual RViz screenshot: {still}")
    return 0


if __name__=="__main__":
    sys.exit(main())

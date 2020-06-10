#!/bin/bash
mkdir video
for c in {cam0,cam1,cam2,cam3}; do
	for t in {heat,density,smoke}; do
		ffmpeg -r 24 -f image2 -pattern_type glob -i "$c/$t/*?png" -vcodec libx265 -crf 20 -pix_fmt yuv420p ./video/v4_"$c"_"$t".mp4
	done
done

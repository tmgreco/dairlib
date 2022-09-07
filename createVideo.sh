 ffmpeg -r 30 -i frame_%07d.tiff \
           -vcodec libx264 \
           -preset slow \
           -crf 18 \
           -pix_fmt yuv420p \
           "${1}.mp4"
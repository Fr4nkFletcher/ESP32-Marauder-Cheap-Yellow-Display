Since the full-res image viewer (project vista) streams the whole image every frame, there is nothing stopping you from streaming a *different* image each frame, and ending up with a very high frame rate, high resolution video. Unfortunately, you will quickly run up against flash storage size limitations.


Save a video such as the Amiga bouncing ball demo (you can find HD renditions of this on Youtube) as `src.mp4`, and run `mkframes.sh`. This exports the first 1 second of src.mp4 as a 20 frame animation, and packs it into a UF2 file, containing around 12 MB of raw frames. *This takes a long time!* You can then change `FRAMES_PER_IMAGE` and `N_IMAGES` in `main.c` to get the whole animation playing at e.g. 20 FPS (can also play it at 60 FPS just fine, it just looks silly).

The creator of the HD juggling ball demo did an excellent writeup [here](https://meatfighter.com/juggler/). They provided source code for their ray tracer, so it should be possible to get better video quality by running their code to get the raw frames, rather than cropping and exporting a YouTube video!

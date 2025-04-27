Find an HD rendition of bad apple, save as `src.mkv`, and then run

```bash
./mkframes.sh
```

I used this version: [https://www.youtube.com/watch?v=V-qbHlH490A](https://www.youtube.com/watch?v=V-qbHlH490A)

This will eventually produce a file called `pack.uf2`, and flash it to your RP2040-based board if one is attached. Currently around a minute of video fits into flash.

You'll need to have the following installed and in your PATH:

- ffmpeg
- Python 3 + PIL
- uf2conv with `rp2040` family


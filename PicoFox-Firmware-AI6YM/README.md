# PicoFox – FIRMWARE

A couple notes about the firmware.

To build this, follow the instructions in the main readme.md but you also need the board files from here: https://github.com/ai6ym/arduino-pico

You also need to manually update the python to the latest (as of this commit, that was 3.14.4) as there is a bug with the embeded version that breaks for fat16. You'll need to modify packages/package.pico.indel.template.json to point to a copy of the embedable files from python with no folders. 

I ended up doing this, then having to manually edit a bunch of files and it was a real PITA, IMO, but I'm not a coder so YMMV.

Therefore if you want to build the files on your on, have at it. I'm going to provide the .uf2 files of AI6YM's version pre-compled, and my version so you can flip back and forth.

-Trevor KG6MDW
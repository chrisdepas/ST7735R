__Fork of https://github.com/juj/ST7735R__

## Changes Overview
- Shortened function names
- Allow defining pins etc from arduino, before #include
- Consolidated into single file
- Removed SDCard stuff
- Added `Display_Clear(r, g, b)` (Sets the entire screen to a single color)
- Added ST7735R_MIN_X, ST7735R_MIN_Y, ST7735R_MAX_X, ST7735R_MAX_Y (Helper definitions for valid x,y ranges)

## Name Changes

| Old                          | New                        |
| ---------------------------- | -------------------------- |
| (None)                       | DisplayClear               |
| ST7735R_BEGIN_TRANSACTION    | DisplayBeginTransaction    |
| ST7735R_END_TRANSACTION      | DisplayEndTransaction      |
| ST7735R_SendCommandList      | DisplaySendCommandList     |
| ST7735R_Begin                | DisplayBegin               |
| ST7735R_Line                 | DisplayLine                |
| ST7735R_DrawMonoSprite       | DisplayDrawMonoSprite      |
| ST7735R_SendCommandList      | DisplaySendCommandList     |
| ST7735R_BeginRect            | DisplayBeginRect           |
| ST7735R_PushPixel            | DisplayPushPixel           |
| ST7735R_EndDraw              | DisplayEndDraw             |
| ST7735R_BeginPixels          | DisplayBeginPixels         |
| ST7735R_Pixel                | DisplayPixel               |
| ST7735R_PushPixel_U16        | DisplayPushPixel_U16       |
| ST7735R_FillRect             | DisplayFillRect            |
| ST7735R_Circle               | DisplayCircle              |
| ST7735R_DrawText             | DisplayDrawText            |
| ST7735R_FilledCircle         | DisplayFilledCircle        |
| ST7735R_DrawMonoSprite       | DisplayDrawMonoSprite      |
| SDCard_GetFileStartingBlock  | Removed                    |
| ST7735R_Draw565              | Removed                    |
| ST7735R_DrawBMP              | Removed                    |
| LoadBMPImage                 | Removed                    |

_______________________________________________________________________________

__Original Readme (Without test related info):__

This repository contains a fast low level Arduino Uno compatible graphics library for the 160x128 pixel 16-bit color TFT LCD display that uses the ST7735R chip. It was written as a programming exercise after realizing that the built-in "Adafruit" TFT library that Arduino ships with is very slow. I wanted to display some slideshow images on the TFT using an Arduino, but unfortunately the built-in libraries take about 2.9 seconds(!) to draw a single 160x128 image on the screen from the SD card. With this library, it is possible to draw an image in about 188 milliseconds, which is a considerable 15.2x performance increase! The aim of this code is to squeeze every individual clock cycle out of the drawing routines to see how fast it is possible to drive the ST7735R display. If you can improve the code here, please let me know.

#### Rendering features

This library offers (almost) drop-in replacements for most built-in TFT library methods, and comes with the following functions:
  - Draw batches of individual pixels with a sequence of `ST7735R_BeginPixels()`, `ST7735R_Pixel()` and `ST7735R_EndDraw()` functions.
  - Fill solid rectangles with the `ST7735R_FillRect()` function.
  - Stream rectangles of custom content with a sequence of `ST7735R_BeginRect()`, `ST7735R_PushPixel()` and `ST7735R_EndDraw()` functions.
  - Draw horizontal lines with the function `ST7735R_HLine()` and vertical lines with the function `ST7735R_VLine()`.
  - Draw arbitrary sloped lines with the function `ST7735R_Line()`.
  - Draw hollow circles with the function `ST7735R_Circle()`.
  - Draw filled circles with the function `ST7735R_FilledCircle()`.
  - Draw 1-bit monochrome sprites from PROGMEM with the function `ST7735R_DrawMonoSprite()`.
  - Draw text stored in PROGMEM with the function `ST7735R_DrawText()`. One font file `"monaco_font.h"` is provided, create more with the `ttffont_to_cppheader.py` tool.
  - Draw 24-bit bottom-up .bmp images with the functions `LoadBMPImage()`, `SDCard_GetFileStartingBlock()` and `ST7735R_DrawBMP()`. (requires hacking of the built-in SD card library to enable raw FAT32 block streaming).
  - Draw pre-prepared raw 16-bit images (files formatted with suffix .565) with the functions `SDCard_GetFileStartingBlock()` and `ST7735R_Draw565()`. (requires hacking of the built-in SD card library to enable raw FAT32 block streaming). Use the python tool `image_to_rgb565.py` to convert images to this format.

The library has only been used on an Arduino Uno with 8MHz hardware SPI mode, and contains several assumptions that might not hold when transferring the configuration to other setups. Don't use blindly without understanding the internals of the code!

#### References

 - The display: http://www.arduino.cc/en/Guide/TFT (bought from http://www.verkkokauppa.com/fi/product/30873/dsgrv/Arduino-1-77-TFT-LCD-Screen-naytto-Arduino-kehitysalustoille)
 - Wiring instructions at http://www.arduino.cc/en/Guide/TFTtoBoards
 - Data sheet at http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf
 - Atmel AVR 8 bit instruction set documentation: http://www.atmel.com/images/doc0856.pdf
 - Requires manual pin to port+bit mapping configuration. This is useful: http://pighixxx.com/unov3pdf.pdf

#### License and Usage

Unless otherwise stated in individual files, all code is released to public domain. Do whatever you wish with it. This repository is a result of some **recreational hacking** activity, rather than a mission to build a stable and maintained library, so expect the maturity to be as such.

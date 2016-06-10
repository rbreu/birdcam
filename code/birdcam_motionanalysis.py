#!/usr/bin/env python2

from astral import Astral
from collections import deque
import datetime
import io
import logging
import matplotlib.image
import numpy
import os
import picamera
import picamera.array
from PIL import Image, ImageChops
import time


############################################################
# Config

DELAY_AFTER_HIT = 0
LOGLEVEL = 'INFO'
MOTION_THRESHOLD = 5
SLEEP_WHEN_DARK = True
IMAGE_RESOLUTION = (1600, 1200)
#IMAGE_RESOLUTION = (800, 600)
MOTION_RESOLUTION = (800, 600)
SAVE_MOTION_IMAGES = True
CROP_IMAGES = True # Crop to lower half of the image


############################################################
# Setup

logging.basicConfig(level=getattr(logging, LOGLEVEL.upper()),
                    filename=os.path.join('images', 'log.txt'))

a = Astral()
a.solar_depression = 3
location = a['Berlin']

# Cologne:
location.latitude = 50.9534001
location.longitude = 6.9548886
location.elevation = 56


motion_mask = matplotlib.image.imread('motion_mask.png')[..., 0]

if CROP_IMAGES:
    motion_mask[:motion_mask.shape[0]//2][...] = 0



class MotionAnalyser(picamera.array.PiMotionAnalysis):

    FRAMES = 5

    def __init__(self, *args, **kwargs):
        super(MotionAnalyser, self).__init__(*args, **kwargs)
        self.motion = None
        self.last_motions = deque([0] * self.FRAMES, maxlen=self.FRAMES)
        self.last_motionframes = deque(maxlen=self.FRAMES)

    def analyse(self, m):
        data = numpy.sqrt(
            numpy.square(m['x'].astype(numpy.float)) +
            numpy.square(m['y'].astype(numpy.float))
        )
        data = numpy.multiply(data, motion_mask)
        norm = numpy.linalg.norm(data)
        self.last_motions.append(norm)
        if SAVE_MOTION_IMAGES:
            self.last_motionframes.append(data)
        logging.debug(norm)
        if min(self.last_motions) > MOTION_THRESHOLD:
            self.motion = min(self.last_motions)


class AliveMessageWriter:

    def __init__(self):
        self.last_written = None

    def write(self, now, dawn, dusk):
        if self.last_written is None or (now - self.last_written).seconds > 10*60:
            with open(os.path.join('/tmp/', 'alive.txt'), 'w') as f:
                f.write(
                    'Now: {0}, dawn: {1}, dusk: {2}'.format(now, dawn, dusk))
                self.last_written = now


def save_image(camera, motion, resolution, motion_data=None):
    old_res = camera.resolution
    camera.resolution = resolution
    now = datetime.datetime.now()

    fname = '{1:06.0f}-{0}.png'.format(now.strftime('%Y-%m-%d-%H%M%S'), motion)
    fname = os.path.join('images', fname)
    logging.debug('Saving image {0}'.format(fname))
    if CROP_IMAGES:
        stream = io.BytesIO()
        camera.capture(stream, format='png')
        stream.seek(0)
        img = Image.open(stream)
        img = img.crop((0, resolution[1]//2, resolution[0], resolution[1]))
        img.save(fname)
    else:
        camera.capture(fname)
    camera.resolution = old_res
    os.system('chmod 666 {0}'.format(fname))

    if motion_data:
        data = sum(motion_data)
        fname = '{1:06.0f}-{0}-motion.png'.format(
            now.strftime('%Y-%m-%d-%H%M%S'), motion)
        fname = os.path.join('images', fname)
        logging.debug('Saving motion image {0}'.format(fname))
        img = Image.fromarray(data.clip(0, 255).astype(numpy.uint8))
        img = img.resize(MOTION_RESOLUTION)
        if CROP_IMAGES:
            img = img.crop((0, MOTION_RESOLUTION[1]//2,
                            MOTION_RESOLUTION[0], MOTION_RESOLUTION[1]))
        img.save(fname)


def save_calibration_image(camera):
    logging.info('Saving calibration image...')
    stream = io.BytesIO()
    camera.capture(stream, format='png')
    stream.seek(0)
    layer1 = Image.open(stream)
    logging.debug(layer1.size)
    logging.debug(layer1.format)
    logging.debug(layer1.mode)
    layer2 = Image.open('motion_mask.png').convert('RGBA')
    layer2 = layer2.resize(MOTION_RESOLUTION)
    logging.debug(layer2.size)
    logging.debug(layer2.format)
    logging.debug(layer2.mode)
    img = ImageChops.multiply(layer1, layer2)
    img.save(os.path.join('images', 'calibration.png'))



############################################################
# Main loop

logging.info('Initialising camera...')

with picamera.PiCamera() as camera:
    logging.info('Setting up camera...')
    camera.resolution = MOTION_RESOLUTION
    camera.framerate = 15
    camera.start_preview()
    time.sleep(2)
    alive_writer = AliveMessageWriter()

    save_calibration_image(camera)

    logging.info('Starting main loop...')

    while True:
        now = location.tz.localize(datetime.datetime.now())
        dawn = location.dawn()
        dusk = location.dusk()
        logging.debug('Now: {0}, dawn: {1}, dusk: {2}'.format(now, dawn, dusk))
        alive_writer.write(now, dawn, dusk)

        if not SLEEP_WHEN_DARK or (dawn < now < dusk):
            with MotionAnalyser(camera) as analyser:
                logging.debug('Starting recording...')
                camera.start_recording(
                    '/dev/null', format='h264', motion_output=analyser)
                logging.debug('Recording...')
                while True:
                    now = location.tz.localize(datetime.datetime.now())
                    alive_writer.write(now, dawn, dusk)
                    if SLEEP_WHEN_DARK and not (dawn < now < dusk):
                        camera.stop_recording()
                        break
                    if analyser.motion is not None:
                        camera.stop_recording()
                        save_image(camera, analyser.motion, IMAGE_RESOLUTION,
                                   analyser.last_motionframes)
                        break
                    time.sleep(0.1)
        else:
            # wait to wake up
            time.sleep(10)

        time.sleep(DELAY_AFTER_HIT)

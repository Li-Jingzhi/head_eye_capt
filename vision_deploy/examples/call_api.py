"""Run after installation; replace pixels with fresh matched target detections."""
import json
from pathlib import Path

from stereo_robot import StereoRobot

model = StereoRobot()  # Load once, outside your camera/detector loop.
sample = json.loads(Path(__file__).with_name('sample_request.json').read_text())
sample.pop('id')
result = model.predict(**sample)
print(json.dumps(result, indent=2))
print('Target XYZ:', result['xyz'], result['units'], 'in', result['frame'])

# In your live loop:
# result = model.predict([uL, vL, uR, vR], arm='right', units='mm')
# After configuring a measured base transform, enforce actual base coordinates:
# result = model.predict(pixels, arm='right', units='mm', require_base=True)
# The bundled left calibration is independent; arm selects the output frame.
left = model.predict(sample['pixels'], arm='left', units='mm')
print('Same target in left MDH parent:', left['xyz'])

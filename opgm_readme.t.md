# OPGM openpilot fork

This is a fork of [openpilot](https://github.com/commaai/openpilot/), an open source driver assistance system. Please
consult the official documentation for most questions.

OPGM strives to align as closely to the official openpilot as possible, while adding functionality for GM vehicles that
are officially unsupported. Additionally, OPGM adds some creature comforts and quality of life improvements.

OPGM will be rebased on top of the official openpilot frequently to keep up with upstream changes. The latest sync was
with commit [`{{.Env.COMMIT | strings.Trunc 10}}`](https://github.com/commaai/openpilot/tree/{{.Env.COMMIT}}) from the
master branch. The latest build was done on {{time.Now.Format "January 02, 2006"}}.

## Features
* Support for GM vehicles with LKAS but no ACC
* Pedal interceptor support for longitudinal control
  * Also provides full regen on GM Bolt EV/EUV with ACC (*in process of upstreaming*)
* CC long, aka "[redneck ACC](https://www.youtube.com/watch?v=41wZ1EAmf94)" to automatically adjust nonadaptive cruise
control
* Neural network steering control, credit twilsonco

## Supported vehicles list
OPGM is capable of supportng vehicles on the Global A architecture that have factory LKAS. Currently supported vehicles
include:
* All vehicles supported by upstream openpilot
* 2016-2019 Chevrolet Volt
* 2017-2019 Chevrolet Bolt EV
* 2020-2023 Chevrolet Bolt EV/EUV w/o ACC
* Chevrolet Equinox/GMC Terrain w/o ACC
* Chevrolet Tahoe/GMC Yukon w/o ACC
* Chevrolet Suburban w/o ACC

If your vehicle is not on this list, there is a very good chance that already is or can be supported! Reach out to
nworby on discord with questions.

## Installation
### Hardware
OPGM supports the Comma Three development platform; legacy support for Comma Two is not guaranteed. Some older development
branches may work on Comma Two. Use at your own risk.

Verify first that your vehicle has LKAS. Verify as well that it has a forward-facing camera; you can do this by removing
the plastic cover on the windshield behind the rearview mirror. If you see a silver rectangular camera, you're good to go.

If you have vehicle supported by upstream openpilot, buy the corresponding hardware from Comma.

If you have a vehicle without ACC, buy the [Bolt EV/EUV kit from Comma](https://comma.ai/shop/comma-three). If you have
a Bolt EV/EUV, it is **strongly** recommended to purchase a pedal interceptor for the best experience. (You may always
add one later, if you want to try OPGM before committing.)

You may buy a pedal interceptor from the following vendors:
* [TinyBear](https://www.etsy.com/listing/952895642/openpilot-comma-pedal-non-customizable?variation0=3013902165)

### Software
For the latest stable build, use the install URL: `installer.comma.ai/opgm/build`

Ensure that your car is *completely* powered off during software installation, otherwise you may get a "no panda" error.
To be sure that your car is completely powered down:
1. Turn the car on and off
2. Open and close the driver's door
3. Wait 5 minutes

## Discussion
Come join us on the OPGM channel in the [openpilot community discord](https://discord.gg/KGWEdwSnCU)!

### Contributing
Feel free to open a pull request against the `dev` branch.

### Credits
* [comma.ai](https://comma.ai) for openpilot
* jshuler
* nworby
* twilsonco
* k1mu
* kliu

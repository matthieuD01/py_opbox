# README

The OpBox is a compact and easy-to-setup ultrasound test device that combines a pulser and a oscilloscope.

There is a webapp to use the OpBox remotely, when it's connected to its Raspberry Pi (see [Smush OpBox](https://gogs.ceec.echem.io/matthieu/smush_opbox)).  
However, you might need to use the OpBox on your local computer, via USB, and record experiments with it. That's what this repo is for.

## How to use
After cloning the repo to your folder and installing the packages in `requirements.txt` (preferably in a dedicated Python env) (see How to Install section below), simply connect the OpBox to your computer via USB, and run the scripts below like you would with the Epoch:
- `pulse.py` : fetches one waveform and plots it. Use it to see what your signal looks like.
- `mission_control.py`: records a waveform (averaged over 64 waveforms) every 5s and saves it in a Crux database in `./acoustic_data` (it will create the folder)
- `mission_control_with_plot.py`: same as above, but adds a matplotlib window with a graph that updates every time a new waveform is recorded.

The main experiment parameters (range, delay, gain) can be modified directly in those script. For refined control over your experiment, refer to the function `setStandardConfig()` in `opbox.py`. Once you've identified which functions to call to modify your target parameter, call it in your main script (`mission_control.py`) right after `opbox = opbox.Opbox_v21(verbose=False)`.

You can verify that the correct settings are applied by calling `registers = opbox.read_all_parameters()`. It will read and print the state of all the registers the OpBox has to offer. You can then refer to `OPBOX-2v2 - Register Description.pdf` to see what it means.

NB: you can also run a copy of `mission_control_with_plot.py` with database recording off and a delay between measurement of 0.5s to adjust your setup.

## How to install
It is best practice to create a dedicated Python environment to ensure no issues with package versions. This demonstrates the typical sequences of commands you might want to use in Windows using Conda, adapt this to your own setup (e.g. `venv`).
- Open cmd or whichever CLI you use, and navigate to the folder you want to setup this repo:

    `cd path/to/where/repo/will/install`
- Do 'git clone https://gogs.ceec.echem.io/matthieu/py_opbox.git'
- Navigate inside the folder:

    `cd py_opbox`
- Create a new Python environment. With conda: 

    `conda create -n opbox_env python=3.11`    (Create a new env with Python 3.11 or above) 

    `conda activate`    (Activates the environment)

    `pip install -r requirements.txt`   (Install all required packages as listed in requirements.txt)
- Now, you can run `pulse.py`, `mission_control.py`, `mission_control_with_plot.py` as you wish. I recommend opening them in your favorite IDE (making sure your conda env is activated) to modify the experiment parameters.

I need to add the correct USB DLL directly in the repo to avoid any issues. 
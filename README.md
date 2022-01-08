# highway-env

<!---[![build](https://github.com/eleurent/highway-env/workflows/build/badge.svg)](https://github.com/eleurent/highway-env/actions?query=workflow%3Abuild)
<>[![Documentation Status](https://readthedocs.org/projects/highway-env/badge/?version=latest)](https://highway-env.readthedocs.io/en/latest/?badge=latest)
<>[![Downloads](https://img.shields.io/pypi/dm/highway-env)](https://pypi.org/project/highway-env/)
<>[![Codacy Badge](https://api.codacy.com/project/badge/Grade/63847d9328f64fce9c137b03fcafcc27)](https://app.codacy.com/manual/eleurent/highway-env?utm_source=github.com&utm_medium=referral&utm_content=eleurent/highway-env&utm_campaign=Badge_Grade_Dashboard)
<>[![Coverage](https://codecov.io/gh/eleurent/highway-env/branch/master/graph/badge.svg)](https://codecov.io/gh/eleurent/highway-env)
<>[![GitHub contributors](https://img.shields.io/github/contributors/eleurent/highway-env)](https://github.com/eleurent/highway-env/graphs/contributors)
<>[![Environments](https://img.shields.io/github/search/eleurent/highway-env/import%20filename:*_env%20path:highway_env/envs?label=environments)](#the-environments) --->

A collection of environments for *autonomous driving* and tactical decision-making tasks. This Library was forked from https://github.com/eleurent/highway-env and modified for our specific interests.

To use the library in Anaconda use the comand line ```conda create --name av_env --file av-env.txt``` to create a new conda environment with the prerequisites for the project.

<p align="center">
    <img src="https://raw.githubusercontent.com/eleurent/highway-env/master/../gh-media/docs/media/highway-env.gif?raw=true"><br/>
    <em>An episode of one of the environments available in highway-env.</em>
</p>

<!---
## [Try it on Google Colab! ![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](scripts) --->

## The environments

### Highway

```python
env = gym.make("highway-v0")
```

In this task, the ego-vehicle is driving on a multilane highway populated with other vehicles.
The agent's objective is to reach a high speed while avoiding collisions with neighbouring vehicles. Driving on the right side of the road is also rewarded.

<p align="center">
    <img src="https://raw.githubusercontent.com/eleurent/highway-env/master/../gh-media/docs/media/highway.gif?raw=true"><br/>
    <em>The highway-v0 environment.</em>
</p>

A faster variant, `highway-fast-v0` is also available, with a degraded simulation accuracy to improve speed for large-scale training.

### Roundabout

```python
env = gym.make("roundabout-v0")
```

In this task, the ego-vehicle if approaching a roundabout with flowing traffic. It will follow its planned route automatically, but has to handle lane changes and longitudinal control to pass the roundabout as fast as possible while avoiding collisions.

<p align="center">
    <img src="https://raw.githubusercontent.com/eleurent/highway-env/master/../gh-media/docs/media/roundabout-env.gif?raw=true"><br/>
    <em>The roundabout-v0 environment.</em>
</p>

### Intersection

```python
env = gym.make("intersection-v0")
```

An intersection negotiation task with dense traffic.

<p align="center">
    <img src="https://raw.githubusercontent.com/eleurent/highway-env/master/../gh-media/docs/media/intersection-env.gif?raw=true"><br/>
    <em>The intersection-v0 environment.</em>
</p>

## Usage
First download the repo, and open the folder ```highway-env```

Then create the conda environment:
```conda create --name av_env --file av-env.txt```
This will create a conda environment with all the required libraries.

Activate conda environment:
```conda activate av_env```

Register Roundabout environment in Gym:
```python highway_env/envs/roundabout_env.py```

For train the model:
```python Training_ReinforcementLearning.py```

For test the model:
```python ReinforcementLearning.py```

## Documentation

Read the official [documentation online](https://highway-env.readthedocs.io/).

## Citing

If you use the project in your work, please consider citing the original repo with:
```bibtex
@misc{highway-env,
  author = {Leurent, Edouard},
  title = {An Environment for Autonomous Driving Decision-Making},
  year = {2018},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/eleurent/highway-env}},
}
```

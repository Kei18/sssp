MRMP
===

A private research repo for multi-robot motion planning (MRMP), written in Julia (≥v1.6).


## Demo

![](./assets/point-robot.gif)
![](./assets/arm.gif)

## Setup

```sh
git clone https://github.com/Kei18/mrmp.git
cd mrmp
julia --project=.
(MRMP) pkg> instantiate
```

## Usage

#### Open JupyterLab

```sh
julia --project=. -e "using IJulia; jupyterlab()"
```


## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at the Tokyo Institute of Technology, interested in controlling multiple moving agents.

## Reference

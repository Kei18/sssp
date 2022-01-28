MRMP
===

[![Run test](https://github.com/Kei18/mrmp/actions/workflows/test.yml/badge.svg?branch=dev)](https://github.com/Kei18/mrmp/actions/workflows/test.yml)

A private research repo for multi-robot motion planning (MRMP), written in Julia (≥v1.6), tested on MacOS-10.15.

## TODO
- [ ] hypra search
  - [ ] baselines
    - [x] point2d
    - [x] point3d
    - [x] line2d
    - [x] arm22
    - [ ] arm33
    - [ ] snake2d
    - [ ] dubins2d
  - [ ] proposal
    - [ ] point2d
    - [ ] point3d
    - [ ] line2d
    - [ ] arm22
    - [ ] arm33
    - [ ] snake2d
    - [ ] dubins2d
  - [ ] scalability test

## Demo

![](./assets/point-robot.gif)
![](./assets/arm.gif)

## Setup

```sh
git clone https://github.com/Kei18/mrmp.git
cd mrmp
julia --project=. -e 'using Pkg; instantiate()'
```

## Usage

#### Open JupyterLab

```sh
julia --project=. -e "using IJulia; jupyterlab()"
```

#### Test
```sh
julia --project=. -e 'using Pkg; test()'
```

#### Format
```sh
julia --project=. -e 'using JuliaFormatter; format(".")'
```

#### Hyperparameter Optimization
```sh
julia --project=. --threads=auto
> include("./scripts/hypraopt.jl")
> @time main("./scripts/config/hypra/params.yaml", "./scripts/config/eval/point2d.yaml")
```


## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at the Tokyo Institute of Technology, interested in controlling multiple moving agents.

## Reference

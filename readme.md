MRMP
===
[![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENCE.txt)
[![CI](https://github.com/Kei18/mrmp/actions/workflows/ci.yaml/badge.svg?branch=dev)](https://github.com/Kei18/mrmp/actions/workflows/ci.yaml)

A private research repo for multi-robot motion planning (MRMP), written in Julia (≥v1.6), tested on MacOS-10.15.

## TODO
- [ ] hypra search
  - [x] baselines
    - [x] point2d
    - [x] point3d
    - [x] line2d
    - [x] capsel3d
    - [x] arm22
    - [x] arm33
    - [x] dubins2d
    - [x] snake2d
  - [ ] proposal
    - [x] point2d
    - [x] point3d
    - [x] line2d
    - [x] capsel3d
    - [x] arm22
    - [x] arm33
    - [x] dubins2d
    - [x] snake2d
    - [ ] point2d (scalability)
- [ ] main result (need re-experiment)
  - [ ] point2d
  - [ ] point3d
  - [ ] line2d
  - [x] capsel3d
  - [ ] arm22 (need re-experiment)
  - [ ] arm33
  - [x] dubins2d
  - [x] snake2d
- [ ] scalability test (point2d)
  - [ ] CBS
  - [ ] planner3
- [x] ablation study (need re-experiment)
  - [x] point2d
  - [x] arm22
  - [x] snake2d
- [ ] robot demo

## Demo

![](./assets/point-robot.gif)
![](./assets/arm.gif)

## Setup

```sh
git clone https://github.com/Kei18/mrmp.git
cd mrmp
julia --project=. -e 'using Pkg; Pkg.instantiate()'
```

## Usage

#### Open JupyterLab

```sh
julia --project=. -e "using IJulia; jupyterlab()"
```

#### Test
```sh
julia --project -e 'using Pkg; Pkg.test()'
```

#### Format
```sh
julia --project=. -e 'using JuliaFormatter; format(".")'
```

#### Hyperparameter Optimization with [Hyperopt.jl](https://github.com/baggepinnen/Hyperopt.jl)
```sh
julia --project=. --threads=auto
> include("./scripts/hypraopt.jl")
> @time main("./scripts/config/hypra/params.yaml", "./scripts/config/eval/point2d.yaml")
```

#### Evaluate Algorithms

```sh
julia --project=. --threads=auto
> include("./scripts/eval.jl")
> @time main("./scripts/config/eval/point2d.yaml", "time_limit=300")
```

The script is inspired by [Hydra](https://hydra.cc/).

#### Scalability Test
```sh
julia --project=. --threads=auto
> include("./scripts/eval.jl")
> @time foreach(N -> main("./scripts/config/eval/point2d_many.yaml", "instance.N=$N"), 10:10:50)
```

#### Ablation Study
```sh
julia --project=. --threads=auto
> include("./scripts/eval.jl")
> @time main("./scripts/config/eval/point2d_ablation.yaml")
> @time main("./scripts/config/eval/arm22_ablation.yaml")
> @time main("./scripts/config/eval/snake2d_ablation.yaml")
```

#### Planning Server (used in robot demo)
```sh
julia --project=.
> include("./scripts/server.jl")
> # To close the server, just perform `close()`
```

## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at the Tokyo Institute of Technology, interested in controlling multiple moving agents.

## Reference

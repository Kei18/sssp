MRMP
===

## TODO
- [ ] Line2d, visualization

---

[![Run test](https://github.com/Kei18/mrmp/actions/workflows/test.yml/badge.svg?branch=dev)](https://github.com/Kei18/mrmp/actions/workflows/test.yml)

A private research repo for multi-robot motion planning (MRMP), written in Julia (≥v1.6), tested on MacOS-10.15.



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

#### Test
```sh
julia --project=. -e 'using Pkg;Pkg.test()'
```

#### Format
```sh
julia --project=. -e 'using JuliaFormatter;format("./src/");format("./example/")'
```


## Licence
This software is released under the MIT License, see [LICENSE.txt](LICENCE.txt).

## Author
[Keisuke Okumura](https://kei18.github.io) is a Ph.D. student at the Tokyo Institute of Technology, interested in controlling multiple moving agents.

## Reference

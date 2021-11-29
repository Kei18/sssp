export CircleObstacle2D

abstract type Obstacle end
struct CircleObstacle2D
    x::Float64
    y::Float64
    r::Float64
end

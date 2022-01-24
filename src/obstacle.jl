abstract type Obstacle end
abstract type CircleObstacle <: Obstacle end

struct CircleObstacle2D <: CircleObstacle
    x::Float64
    y::Float64
    r::Float64
end

struct CircleObstacle3D <: CircleObstacle
    x::Float64
    y::Float64
    z::Float64
    r::Float64
end

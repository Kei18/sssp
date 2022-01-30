"""definition of obstacles"""

abstract type Obstacle end
abstract type CircleObstacle <: Obstacle end

"""circle obstacle in 2D space"""
struct CircleObstacle2D <: CircleObstacle
    x::Float64
    y::Float64
    r::Float64
end

"""circle obstacle in 3D space"""
struct CircleObstacle3D <: CircleObstacle
    x::Float64
    y::Float64
    z::Float64
    r::Float64
end

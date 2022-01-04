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

to_string(o::CircleObstacle2D) = @sprintf("(%.4f, %.4f), rad: %.4f", o.x, o.y, o.r)
to_string(o::CircleObstacle3D) = @sprintf("(%.4f, %.4f, %.4f), rad: %.4f", o.x, o.y, o.z, o.r)

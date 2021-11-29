using MRMP: StatePoint2D, CircleObstacle2D, search, Node
using Test

# simple example
@test 1 + 1 == 2

@testset "point2d" begin
    include("point2d.jl")
end

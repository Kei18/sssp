@testset "tpg" begin
    include("../example/smoothing_point2d.jl")
    @test solution_after != nothing
end

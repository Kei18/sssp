@testset "refine" begin
    include("../example/refine_point2d.jl")
    @test solution_after != nothing
end

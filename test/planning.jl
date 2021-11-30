@testset "point2d" begin
    include("../example/point2d.jl")
    @test S != nothing
end

@testset "point3d" begin
    include("../example/point3d.jl")
    @test S != nothing
end

@testset "line2d" begin
    include("../example/line2d.jl")
    @test S != nothing
end

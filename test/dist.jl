using MRMP: dist

@testset "dist" begin
    p1_1 = [-1.0, -2.0]
    p1_2 = [1.0, 2.0]
    p2_1 = [1.0, -2.0]
    p2_2 = [-1.0, 2.0]

    @test dist(p1_1, p1_2, p2_1, p2_2) == 0
end

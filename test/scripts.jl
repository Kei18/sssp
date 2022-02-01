@testset "eval" begin
    include("../scripts/eval.jl")
    args_common = ["num_instances=1", "time_limit=1", "root=./local/"]
    yaml_files = [
        "../scripts/config/eval/point2d.yaml"
        "../scripts/config/eval/point3d.yaml"
        "../scripts/config/eval/line2d.yaml"
        "../scripts/config/eval/arm22.yaml"
        "../scripts/config/eval/arm33.yaml"
        "../scripts/config/eval/snake2d.yaml"
        "../scripts/config/eval/dubins2d.yaml"
        "../scripts/config/eval/capsel3d.yaml"
    ]
    foreach(e -> main(e, args_common...), yaml_files)
end

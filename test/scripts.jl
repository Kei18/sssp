@testset "eval" begin
    include("../scripts/eval.jl")
    args_common = ["num_instances=1", "time_limit=1", "root=./local/"]
    yaml_file = "../scripts/config/eval/point2d.yaml"
    main(yaml_file, args_common...)
end

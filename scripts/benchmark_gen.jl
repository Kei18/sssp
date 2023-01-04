"""
benchmark generation used in experiment
"""

using MRMP
import YAML
import Dates
import JLD2
import Random: seed!
import Base.Threads

include("./utils.jl")

function main(args...; kwargs...)
    # load experimental setting
    config = get_config(args...; kwargs...)
    num_instances = get(config, "num_instances", 10)
    typeof(num_instances) != Int && (num_instances = parse(Int, num_instances))
    flg_save_fig = get(config, "save_fig", true)
    typeof(flg_save_fig) != Bool && (flg_save_fig = parse(Bool, flg_save_fig))
    flg_save_fig &= (Threads.nthreads() == 1)

    # prepare directory
    date_str = replace(string(Dates.now()), ":" => "-")
    root_dir = joinpath(pwd(), "..", "data", "benchmark", date_str)
    !isdir(root_dir) && mkpath(root_dir)

    # save configuration file
    save_config(config, root_dir, date_str)

    # prepare generator
    target = Meta.parse(config["generator"]["target"])
    params = Dict()
    foreach(e -> params[Symbol(first(e))] = last(e), config["generator"]["params"])
    generator = (args..., ; kwargs...) -> eval(target)(args...; params..., kwargs...)

    # generate instances
    I = Vector{Any}(undef, num_instances)
    cnt_fin = Threads.Atomic{Int}(0)
    r = (x) -> round(x, digits = 3)  # round
    t_start = Base.time_ns()

    Threads.@threads for k = 1:num_instances
        seed!(k)
        ins = generator()
        flg_save_fig &&
            MRMP.plot_instance!(ins...; filename = joinpath(root_dir, "$(k).png"))
        I[k] = ins
        Threads.atomic_add!(cnt_fin, 1)
        print(
            "\r" *
            "$(r((Base.time_ns() - t_start) / 1.0e9)) sec" *
            "\t$(cnt_fin[])/$(num_instances) " *
            "($(r(cnt_fin[]/num_instances*100))%)" *
            " tasks have been finished",
        )
    end

    # save instances
    benchmark_file = joinpath(root_dir, "instances.jld2")
    JLD2.save(benchmark_file, "instances", I)
    println("\nbenchmark file was saved in $(benchmark_file)")
    postprocessing(config)
end

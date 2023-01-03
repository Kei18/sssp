"""
benchmark generation used in experiment
"""

using MRMP
import YAML
import Dates
import JLD2
import Random: seed!
import Base.Threads


function main(config_file::String)
    # load experimental setting
    config = YAML.load_file(config_file)
    num_instances = get(config, "num_instances", 10)
    flg_save_fig = get(config, "save_fig", true)

    # prepare directory
    date_str = replace(string(Dates.now()), ":" => "-")
    root_dir = joinpath(pwd(), "..", "data", "benchmark", date_str)
    !isdir(root_dir) && mkpath(root_dir)

    # save configuration file
    io = IOBuffer()
    versioninfo(io, verbose = true)
    additional_info = Dict(
        "git_hash" => read(`git log -1 --pretty=format:"%H"`, String),
        "date" => date_str,
        "nthreads" => Threads.nthreads(),
        "env" => String(take!(io)),
    )
    YAML.write_file(joinpath(root_dir, "config.yaml"), merge(config, additional_info))

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
        if flg_save_fig && Threads.nthreads() == 1
            MRMP.plot_instance!(ins...; filename = joinpath(root_dir, "$(k).png"))
        end
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
end

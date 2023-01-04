import YAML
using MRMP

function get_config(args...; kwargs...)
    dict = merge(
        map(
            arg -> begin
                isfile(arg) && return YAML.load_file(arg)
                typeof(arg) == String &&
                    return Dict(first(split(arg, "=")) => last(split(arg, "=")))
                typeof(arg) == Dict && return arg
                Dict()
            end,
            args,
        )...,
    )
    foreach(e -> dict[String(first(e))] = last(e), kwargs)
    return dict
end

function save_config(config::Dict, root_dir::String, date_str::String)
    io = IOBuffer()
    versioninfo(io, verbose=true)
    git_hash = try
        read(`git log -1 --pretty=format:"%H"`, String)
    catch
        ""
    end
    additional_info = Dict(
        "git_hash" => git_hash,
        "date" => date_str,
        "nthreads" => Threads.nthreads(),
        "env" => String(take!(io)),
    )
    YAML.write_file(joinpath(root_dir, "config.yaml"), merge(config, additional_info))
end

function get_solver_args(ins)
    config_init, config_goal, obstacles, ins_params... = ins
    connect = gen_connect(config_init[1], obstacles, ins_params...)
    collide = gen_collide(config_init[1], ins_params...)
    check_goal = gen_check_goal(config_goal)
    return [config_init, config_goal, connect, collide, check_goal]
end

function send_slack_msg(args...; kwargs...)
    command = vcat("slack-msg", args..., kwargs...)
    try
        run(pipeline(`$command`))
    catch e
        nothing
    end
    nothing
end

function postprocessing(config::Dict)
    for func_info in get(config, "postprocessing", [])
        params = Dict()
        foreach(e -> params[Symbol(first(e))] = last(e), func_info["params"])
        eval(Meta.parse(func_info["target"]))(; params...)
    end
end

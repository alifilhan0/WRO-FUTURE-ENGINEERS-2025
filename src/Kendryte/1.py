import os
import argparse
import nncase

def read_model_file(model_file):
    with open(model_file, 'rb') as f:
        return f.read()

def main():
    parser = argparse.ArgumentParser(prog="nncase")
    parser.add_argument("--target", type=str, required=True, help="Target to run (e.g. k230)")
    parser.add_argument("--model", type=str, required=True, help="Path to TFLite model")
    args = parser.parse_args()

    dump_dir = "tmp/steering_tflite"
    os.makedirs(dump_dir, exist_ok=True)

    # Compile options
    compile_options = nncase.CompileOptions()
    compile_options.target = args.target
    compile_options.dump_ir = True
    compile_options.dump_asm = True
    compile_options.dump_dir = dump_dir

    # Input configuration (adjust if your model input shape differs!)
    compile_options.input_shape = [1,4]  # NHWC format for TFLite
    compile_options.input_type = "float32"          # steering_model uses float32
    compile_options.output_type = "float32"
    compile_options.input_layout = "NHWC"

    # Compiler
    compiler = nncase.Compiler(compile_options)

    # Import the TFLite model
    print(f"Importing model: {args.model}")
    model_content = read_model_file(args.model)
    import_options = nncase.ImportOptions()
    compiler.import_tflite(model_content, import_options)

    # Compile
    print("Compiling...")
    compiler.compile()

    # Export kmodel
    output_path = os.path.join(dump_dir, "steering.kmodel")
    print(f"Saving kmodel to: {output_path}")
    kmodel = compiler.gencode_tobytes()
    with open(output_path, "wb") as f:
        f.write(kmodel)

    print(f"✅ Successfully converted {args.model} → {output_path}")
    print(f"KModel size: {len(kmodel)/1024:.2f} KB")

if __name__ == "__main__":
    main()

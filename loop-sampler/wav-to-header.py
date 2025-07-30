import gradio as gr
import numpy as np
from scipy.io import wavfile
from io import BytesIO
import tempfile
import os

def convert_wavs_to_header(files, pwm_resolution=4096):
    header_lines = []
    sample_definitions = []
    sample_index = 0

    header_lines.append(f"#define NUM_SAMPLES {len(files)}\n\n")
    header_lines.append("struct SampleData {\n"
                        "    const uint16_t index;\n"
                        "    const int16_t* data;\n"
                        "    const uint32_t size;\n"
                        "    const char* name;\n"
                        "};\n\n")

    array_blocks = []
    for file in files:
        rate, data = wavfile.read(file.name)

        if data.ndim == 2:
            data = data.mean(axis=1)

        data = data.astype(np.float32)
        peak = np.max(np.abs(data))
        if peak > 0:
            data /= peak

        scaled = np.clip(((data + 1) / 2 * (pwm_resolution - 1)).astype(np.int16), 0, 32767)

        array_name = os.path.splitext(os.path.basename(file.name))[0].replace(" ", "_")
        array_lines = [f"const int16_t {array_name}[] PROGMEM = {{"]
        array_lines += [", ".join(map(str, scaled[i:i+10])) for i in range(0, len(scaled), 10)]
        array_lines.append("};\n")

        array_blocks.append("\n".join(array_lines))
        sample_definitions.append(f'    {{ {sample_index}, &{array_name}[0], {len(scaled)}, "{array_name}" }},')
        sample_index += 1

    sample_def_block = "SampleData sample[NUM_SAMPLES] = {\n" + "\n".join(sample_definitions) + "\n};\n\n"
    header_output = "".join(header_lines) + sample_def_block + "\n".join(array_blocks)

    with tempfile.NamedTemporaryFile(delete=False, suffix=".h", mode="w", encoding="utf-8") as f:
        f.write(header_output)
        return f.name

gr.Interface(
    fn=convert_wavs_to_header,
    inputs=[
        gr.File(file_types=[".wav"], file_count="multiple", label="Upload .wav files"),
        gr.Number(label="PWM Resolution", value=4096)
    ],
    outputs=gr.File(label="Download Header File"),
    title="WAV to Header Converter",
    description="Drag and drop WAV files. Outputs a C header with normalized, int16-scaled sample arrays.",
).launch()

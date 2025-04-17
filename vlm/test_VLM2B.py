import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForImageTextToText, BitsAndBytesConfig
from transformers.image_utils import load_image
import gc
import psutil
import os
import time
import numpy as np

def get_gpu_memory():
    """Get the GPU memory usage in MB if available."""
    if torch.backends.mps.is_available():
        # Clear cache to get accurate measurement
        torch.mps.empty_cache()
        # Get memory in bytes, convert to MB
        memory_allocated = torch.mps.current_allocated_memory() / (1024 ** 2)
        memory_reserved = torch.mps.driver_allocated_memory() / (1024 ** 2)
        return {
            'allocated_MB': memory_allocated,
            'reserved_MB': memory_reserved
        }
    return None


def get_cpu_memory():
    """Get the CPU memory usage in MB."""
    process = psutil.Process(os.getpid())
    memory_info = process.memory_info()
    memory_mb = memory_info.rss / (1024 ** 2)  # Convert bytes to MB
    return memory_mb


def log_memory_usage(stage):
    """Log memory usage at a particular stage."""
    cpu_memory = get_cpu_memory()
    gpu_memory = get_gpu_memory()

    print(f"\n----- MEMORY USAGE AT {stage} -----")
    print(f"CPU Memory: {cpu_memory:.2f} MB")

    if gpu_memory:
        print(f"GPU Memory Allocated: {gpu_memory['allocated_MB']:.2f} MB")
        print(f"GPU Memory Reserved: {gpu_memory['reserved_MB']:.2f} MB")
    print("------------------------------\n")

    return {'cpu_mb': cpu_memory, 'gpu': gpu_memory}


# Log initial memory usage
initial_memory = log_memory_usage("INITIAL STATE")

if torch.backends.mps.is_available():
    print("MPS is available.")
    DEVICE = "mps"
elif torch.cuda.is_available():
    print("CUDA is available.")
    DEVICE = "cuda"
else:
    print("CUDA is not available. Using CPU.")
    DEVICE = "cpu"

# Initialize processor and model
model_path = "HuggingFaceTB/SmolVLM2-2.2B-Instruct-mlx"

# quantization_config = BitsAndBytesConfig(
#     load_in_4bit=True,  # Enable 4-bit mode
#     bnb_4bit_use_double_quant=True,  # Further reduces memory
#     bnb_4bit_quant_type="nf4",  # Use Normalized Float 4 (better precision)
#     bnb_4bit_compute_dtype=torch.float16  # Ensures compatibility
# )

processor = AutoProcessor.from_pretrained(model_path)
log_memory_usage("AFTER LOADING PROCESSOR")

'''
model = AutoModelForImageTextToText.from_pretrained(
    model_path,
    torch_dtype=torch.bfloat16,
    _attn_implementation="flash_attention_2"
).to("cuda")
'''
model = AutoModelForImageTextToText.from_pretrained(
    model_path,
    device_map=DEVICE,
).to(DEVICE)
print("Model loaded on", model.device)

log_memory_usage("AFTER LOADING MODEL (CPU)")

start_time = time.time()

# Create input messages
'''
messages = [
    {
        "role": "user",
        "content": [
            {"type": "image", "url": "Cracking.jpg"},
            {"type": "text", "text": "Can you describe this image?"},
        ]
    },
]
'''
messages = [
    {
        "role": "user",
        "content": [
            {"type": "Cracking.jpg"},  # ✅ Indicates that the input includes an image
            {
                "type": "text",
                "text": """You are an expert in structural analysis. Carefully analyze the given image and provide insights in the following structured format:

- **Type of Defect**: (Describe the defect or anomaly present in the image.)
- **Severity Level**: (Estimate the severity from Low, Medium, or High.)
- **Potential Causes**: (Explain what could have led to the defect.)
- **Recommended Actions**: (Suggest appropriate repairs or mitigation steps.)

Now, analyze the image accordingly."""
            }
        ]
    },
]

# Prepare inputs
print("Preparing inputs...")
# Prepare inputs while keeping format
# Prepare inputs with the same format

inputs = processor.apply_chat_template(
    messages,
    add_generation_prompt=True,
    tokenize=True,
    return_dict=True,
    return_tensors="pt",
).to(model.device)

# Convert inputs properly
for key, value in inputs.items():
    if key == "input_ids":
        inputs[key] = value.to(model.device, dtype=torch.long)  # ✅ Keep `input_ids` as `long`
    elif key == "pixel_values":
        inputs[key] = value.to(model.device, dtype=torch.float16)  # ✅ Convert image inputs to `float16`
    else:
        inputs[key] = value.to(model.device, dtype=torch.float32)  # ✅ Other tensors remain `float32`


log_memory_usage("AFTER PREPARING INPUTS")

start = time.time()
generated_ids = model.generate(**inputs, do_sample=False, max_new_tokens=256)
generated_texts = processor.batch_decode(
    generated_ids,
    skip_special_tokens=True,
)
end = time.time()
end_time = time.time()

print("\nGENERATED TEXT:")
print(generated_texts[0])

# Calculate memory increase
final_memory = log_memory_usage("FINAL STATE")

# Print memory increase summary
print("\n----- MEMORY USAGE SUMMARY -----")
print(f"Initial CPU Memory: {initial_memory['cpu_mb']:.2f} MB")
print(f"Final CPU Memory: {final_memory['cpu_mb']:.2f} MB")
print(f"CPU Memory Increase: {final_memory['cpu_mb'] - initial_memory['cpu_mb']:.2f} MB")

delta_time = end_time - start_time
if initial_memory['gpu'] and final_memory['gpu']:
    print(f"Initial GPU Memory Allocated: {initial_memory['gpu']['allocated_MB']:.2f} MB")
    print(f"Final GPU Memory Allocated: {final_memory['gpu']['allocated_MB']:.2f} MB")
    print(f"GPU Memory Increase: {final_memory['gpu']['allocated_MB'] - initial_memory['gpu']['allocated_MB']:.2f} MB")
    print("Total time elpased: ", delta_time)
    print("Generation time: ", end - start)
print("------------------------------")

# Clean up to free memory
del model, processor, inputs, generated_ids
torch.cuda.empty_cache() if torch.cuda.is_available() else None
torch.mps.empty_cache() if torch.backends.mps.is_available() else None
gc.collect()

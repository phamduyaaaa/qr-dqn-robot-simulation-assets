import os

# Đường dẫn tới thư mục chứa file world
world_path = os.path.expanduser("src")
device_name = "kasm-user"
# Lặp qua tất cả các file .world trong thư mục và sửa đường dẫn
for filename in os.listdir(world_path):
    if filename.endswith(".world"):
        file_path = os.path.join(world_path, filename)
        with open(file_path, 'r') as file:
            content = file.read()
        # Thay thế
        content = content.replace('nguyendat',f'{device_name}')
        with open(file_path, 'w') as file:
            file.write(content)
print("done")

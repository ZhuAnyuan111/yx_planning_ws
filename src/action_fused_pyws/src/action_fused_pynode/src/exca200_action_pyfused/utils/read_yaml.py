import yaml

def read_yaml(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)   # 推荐使用 safe_load，更安全
    return data
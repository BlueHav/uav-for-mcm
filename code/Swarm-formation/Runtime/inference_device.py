def select_device(requested,cuda_available):
    if requested not in ('auto','cpu','cuda'):raise ValueError('YOPO_INFERENCE_DEVICE must be auto, cpu or cuda')
    if requested=='cuda' and not cuda_available:raise ValueError('CUDA inference explicitly requested but unavailable')
    return ('cuda' if cuda_available else 'cpu') if requested=='auto' else requested

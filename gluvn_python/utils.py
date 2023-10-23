import queue

def merge_queues(queue1, queue2):
    """
    Merge two queues of vectors where each vector's first element is a timestamp.
    Assumes both input queues are already sorted by timestamp.

    Parameters:
        queue1, queue2: Queues of vectors to merge. Each vector's first element
                       should be a timestamp.
    Returns:
        A new queue containing all vectors from queue1 and queue2, sorted by timestamp.
    """
    merged_queue = queue.Queue()
    
    try:
        item1 = queue1.get_nowait()
    except queue.Empty:
        item1 = None
    
    try:
        item2 = queue2.get_nowait()
    except queue.Empty:
        item2 = None

    # Merge until one queue is empty
    while item1 is not None and item2 is not None:
        if item1[0] < item2[0]:
            merged_queue.put(item1)
            try:
                item1 = queue1.get_nowait()
            except queue.Empty:
                item1 = None
        else:
            merged_queue.put(item2)
            try:
                item2 = queue2.get_nowait()
            except queue.Empty:
                item2 = None
    
    # Add remaining elements (if any) from queue1
    while item1 is not None:
        merged_queue.put(item1)
        try:
            item1 = queue1.get_nowait()
        except queue.Empty:
            item1 = None
    
    # Add remaining elements (if any) from queue2
    while item2 is not None:
        merged_queue.put(item2)
        try:
            item2 = queue2.get_nowait()
        except queue.Empty:
            item2 = None
    
    return merged_queue
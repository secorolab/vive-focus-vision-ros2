#!/usr/bin/env python3
"""Export accepted episodes transactionally using the installed LeRobot environment.

Rebuild a local snapshot so failed encoding cannot corrupt the last good dataset.
Raw episodes remain the source of truth. No model download or Hub upload occurs.
"""
import argparse
import json
import os
from pathlib import Path
import shutil
import uuid


def export(root):
    import numpy as np
    from PIL import Image
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(root).resolve()
    config = json.loads((root / 'session.json').read_text())
    episodes = sorted((root / 'episodes').glob('*/episode.json'), key=lambda p: p.stat().st_mtime_ns)
    if not episodes: raise ValueError('No accepted episodes')
    names = [f'right_joint{i}' for i in range(1, 8)] + ['right_gripper']
    features = {key: dict(dtype='float32', shape=(8,), names=names)
                for key in ('observation.state', 'action')}
    for camera in config['cameras']:
        features[f'observation.images.{camera}'] = dict(dtype='video', shape=(480,640,3), names=['height','width','channels'])
    exports = root / 'exports'
    exports.mkdir(exist_ok=True)
    output = exports / uuid.uuid4().hex
    dataset = None
    try:
        dataset = LeRobotDataset.create('local/openarm_right', fps=config['fps'], root=output,
            features=features, robot_type='openarm_v1_right', vcodec='h264', image_writer_threads=2,
            encoder_threads=2)
        for meta in episodes:
            episode = meta.parent
            info = json.loads(meta.read_text())
            rows = [json.loads(line) for line in (episode / 'frames.jsonl').read_text().splitlines()]
            if len(rows) != info['frames'] or not rows: raise ValueError(f'Invalid episode {episode.name}')
            for index, row in enumerate(rows):
                if row['index'] != index: raise ValueError('Noncontiguous frame indices')
                frame = {'observation.state':np.asarray(row['observation'], dtype=np.float32),
                         'action':np.asarray(row['action'], dtype=np.float32), 'task':config['task']}
                for camera in config['cameras']:
                    with Image.open(episode / camera / f'{index:06d}.jpg') as im:
                        frame[f'observation.images.{camera}'] = np.array(im.convert('RGB'))
                dataset.add_frame(frame)
            dataset.save_episode(parallel_encoding=False)
        dataset.finalize()
        # Validate completed metadata before publishing a new dataset pointer.
        metadata = json.loads((output / 'meta/info.json').read_text())
        if metadata['total_episodes'] != len(episodes): raise ValueError('Export episode count mismatch')
        link = root / 'dataset'
        if link.exists() and not link.is_symlink(): raise ValueError('dataset must be absent or our generated symlink')
        old = link.resolve() if link.is_symlink() else None
        temporary = root / '.dataset-next'
        temporary.unlink(missing_ok=True)
        temporary.symlink_to(output.relative_to(root))
        os.replace(temporary, link)
        if old and old.parent == exports and old != output:
            # Publication already succeeded; cleanup failure must not invalidate it.
            try: shutil.rmtree(old)
            except OSError as exc: print(f'Old export retained: {exc}')
        print(f'Saved {len(episodes)} episodes to {link}', flush=True)
    except Exception:
        if dataset: dataset.finalize()
        if output.exists(): shutil.rmtree(output)
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root')
    export(parser.parse_args().root)

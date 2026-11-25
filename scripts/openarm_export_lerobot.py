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
import tempfile
import uuid


def select_encoder(requested='auto'):
    if requested == 'cpu':
        return 'h264'
    from PIL import Image
    from lerobot.datasets.video_utils import encode_video_frames
    try:
        with tempfile.TemporaryDirectory(prefix='openarm-nvenc-') as directory:
            folder = Path(directory)
            for index in range(3):
                Image.new('RGB', (640, 480), (index*60, 30, 90)).save(folder/f'frame-{index:06d}.png')
            encode_video_frames(folder, folder/'probe.mp4', fps=30,
                                vcodec='h264_nvenc', encoder_threads=2)
            import av
            with av.open(str(folder/'probe.mp4')) as video:
                if sum(1 for _ in video.decode(video=0)) != 3:
                    raise RuntimeError('GPU encoder produced an incomplete probe video')
        return 'h264_nvenc'
    except Exception as error:
        if requested == 'gpu':
            raise RuntimeError(f'NVIDIA video encoding unavailable: {error}') from error
        print(f'NVIDIA encoder unavailable; using CPU H.264: {error}', flush=True)
        return 'h264'


def export_counts(root):
    root = Path(root)
    saved = {p.parent.name for p in (root/'episodes').glob('*/episode.json')}
    exported = set()
    for manifest in (root/'exports').glob('*/source_episodes.json'):
        try:
            ids = json.loads(manifest.read_text())['episodes']
            info = json.loads((manifest.parent/'meta/info.json').read_text())
            if len(ids) == info['total_episodes']:
                exported.update(ids)
        except (OSError, ValueError, KeyError, TypeError):
            continue
    return dict(total=len(saved), exported=len(saved & exported), waiting=len(saved-exported))


def export(root, encoder='auto', latest=False):
    import numpy as np
    from PIL import Image
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(root).resolve()
    config = json.loads((root / 'session.json').read_text())
    episodes = sorted((root / 'episodes').glob('*/episode.json'), key=lambda p: p.stat().st_mtime_ns)
    if not episodes: raise ValueError('No accepted episodes')
    if latest: episodes = episodes[-1:]
    print('Exporting source episodes: ' + ', '.join(p.parent.name for p in episodes), flush=True)
    codec = select_encoder(encoder)
    print(f'Video export: {codec} ({"NVIDIA GPU" if codec == "h264_nvenc" else "CPU"}). Images and data preparation still use CPU.', flush=True)
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
            features=features, robot_type='openarm_v1_right', vcodec=codec, image_writer_threads=2,
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
        (output/'source_episodes.json').write_text(json.dumps(
            dict(episodes=[p.parent.name for p in episodes], encoder=codec), indent=2)+'\n')
        link = root / ('dataset-latest' if latest else 'dataset')
        if link.exists() and not link.is_symlink(): raise ValueError('dataset must be absent or our generated symlink')
        temporary = root / '.dataset-next'
        temporary.unlink(missing_ok=True)
        temporary.symlink_to(output.relative_to(root))
        os.replace(temporary, link)
        print(f'Saved {len(episodes)} episodes to {output}', flush=True)
        print(f'Latest dataset: {link}. Previous exports retained.', flush=True)
    except Exception:
        if dataset: dataset.finalize()
        if output.exists(): shutil.rmtree(output)
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root')
    parser.add_argument('--encoder', choices=('auto', 'gpu', 'cpu'), default='auto',
                        help='Auto tests NVIDIA encoding and falls back to CPU if unavailable')
    parser.add_argument('--latest', action='store_true',
                        help='Export only the newest saved episode; preserve the full dataset pointer')
    args = parser.parse_args()
    export(args.root, args.encoder, args.latest)

#!/usr/bin/env python3
"""Server-only managed script rebuild against the bundled Unity Player APIs.

Does not invoke the Unity Editor or alter scenes, resources or the running Player.
Install the resulting, reviewed DLL through setup.sh before starting a new run.
"""
import argparse,hashlib,json,os,subprocess,tempfile
from pathlib import Path

def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    root=Path(__file__).resolve().parents[1]
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--editor',default=os.environ.get('YOPO_UNITY_EDITOR'),
                    help='Unity Editor executable; uses its bundled standalone Roslyn tools without editor activation')
    args=ap.parse_args()
    if not args.editor:ap.error('Set YOPO_UNITY_EDITOR or pass --editor /path/to/Editor/Unity')
    data=Path(args.editor).resolve().parent/'Data'
    dotnet=data/'NetCoreRuntime/dotnet';compiler=data/'DotNetSdkRoslyn/csc.dll'
    if not dotnet.is_file() or not compiler.is_file():ap.error('Editor distribution lacks the required Roslyn compiler/runtime')
    managed=root/'unity/Build/MountainCity_Data/Managed'
    if not (managed/'UnityEngine.CoreModule.dll').is_file():ap.error('Unpack the bundled Player with setup.sh first')
    project=root/'unity/Project'
    sources=sorted(p for p in (project/'Assets').glob('*.cs'))
    references=sorted(p for p in managed.glob('*.dll') if p.name!='Assembly-CSharp.dll')
    output=root/'assets/unity-managed';output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(dir=output) as temp:
        library=Path(temp)/'Assembly-CSharp.dll'
        command=[str(dotnet),str(compiler),'-noconfig','-nostdlib','-target:library',
                 '-langversion:latest','-deterministic+','-out:'+str(library)]
        command+=['-r:'+str(p) for p in references]
        command+=[str(p.relative_to(project)) for p in sources]
        subprocess.run(command,cwd=project,check=True)
        metadata=dict(method='managed-script-rebuild',scope='existing Player scripts; scenes/resources unchanged',
            compiler_version=subprocess.check_output([str(dotnet),str(compiler),'-version'],text=True).strip(),
            sources={str(p.relative_to(root)):digest(p) for p in sources},
            references={p.name:digest(p) for p in references},assembly_sha256=digest(library))
        library.replace(output/'Assembly-CSharp.dll')
        (output/'build.json').write_text(json.dumps(metadata,indent=2)+'\n')
    sums=root/'assets/SHA256SUMS'
    lines=[line for line in sums.read_text().splitlines() if 'unity-managed/' not in line]
    lines += [digest(output/name)+'  unity-managed/'+name for name in ('Assembly-CSharp.dll','build.json')]
    sums.write_text('\n'.join(lines)+'\n')
    print('Built scripts and refreshed asset hashes. Run setup.sh with no simulation running to install.')

if __name__=='__main__':main()

# Milestone 11: Paper Study and Stress Test Guide

## What this milestone is for

Milestone 11 is where the renderer becomes a small research project.

You already have the useful baseline:

- multiple LAS/LAZ/COPC files can form one dataset;
- tiles are ranked by camera distance;
- a 256 MiB point-buffer budget controls residency;
- unwanted tiles can leave GPU memory safely;
- each resident tile has a CPU-built point hierarchy;
- view-frustum culling and a global draw budget limit submitted points;
- screen-space spacing, hysteresis, and a priority queue choose hierarchy refinement;
- frame, CPU, and GPU timing can be measured.

The next job is to understand one published rendering method, choose one focused idea from it, and compare that idea against this baseline with repeatable evidence.

## The paper

Read **Real-Time Continuous Level of Detail Rendering of Point Clouds** by Markus Schütz, Katharina Krösl, and Michael Wimmer (IEEE VR 2019).

- [Official publication page](https://www.cg.tuwien.ac.at/research/publications/2019/schuetz-2019-CLOD/)
- [Author-hosted preprint PDF](https://www.cg.tuwien.ac.at/research/publications/2019/schuetz-2019-CLOD/schuetz-2019-CLOD-paper_preprint.pdf)
- [Essential source-code samples](https://github.com/m-schuetz/compute_rasterizer/tree/master/compute_loop_lod)

The paper's central idea is continuous, point-wise LOD. It rebuilds a view-dependent downsampled point buffer on the GPU instead of only switching whole hierarchy chunks between fixed detail levels. The result is meant to reduce visible density jumps and popping.

## How to read it without getting buried

### Pass 1: understand the claim

Read the abstract, introduction, figures, and conclusion. Write down:

1. What visual problem are the authors fixing?
2. What does their renderer do differently from discrete, node-based LOD?
3. What result do they claim?

Do not stop to decode every equation or GPU detail during this pass.

### Pass 2: trace one frame

Read the method section and follow one frame from the full point buffer to the final draw:

1. How is the desired density computed for a point or region?
2. Which points survive?
3. Where are surviving points written?
4. How does the draw consume the new buffer?
5. Which work happens in compute shaders and which work happens in the draw pass?

Draw the pipeline on paper. A rough boxes-and-arrows sketch is enough.

### Pass 3: map it to this renderer

Make a two-column list:

| Current renderer | Paper method |
|---|---|
| CPU traverses per-tile octrees | GPU performs view-dependent point selection |
| Whole node ranges are selected | Selection can vary point by point |
| Priority queue spends a global point budget | GPU compaction produces the draw buffer |
| Hysteresis reduces node switching | Continuous sampling reduces density jumps |
| Resident tiles already have GPU point buffers | Paper assumes source points are available to the GPU |

This tells you what can stay and what a research-inspired experiment would replace.

### Pass 4: choose one experiment

Write a one-sentence hypothesis before implementing anything. A good first hypothesis is:

> GPU point-wise selection will make LOD transitions less noticeable than node-wise switching while keeping frame time within the same practical range.

The first experiment should keep the existing tile residency system. Apply the new selection method only to points in resident tiles. This keeps disk streaming, COPC range requests, and a complete renderer rewrite out of the same experiment.

Do not call a simple parent/child cross-fade an implementation of the paper. It can be a useful separate experiment, but label it accurately.

## What to measure

Use the same build, resolution, point budget, camera route, and dataset for both methods.

Record:

- average and worst frame time;
- average CPU render time;
- average GPU render time;
- submitted point count;
- resident tile count and allocated point-buffer memory;
- visible popping or density bands while moving;
- time spent loading and building a hierarchy for a newly wanted tile;
- any allocation, synchronization, or validation warnings.

Capture the same short camera movement for each method. Numbers show cost; a recording shows whether the transition actually looks better.

## Stress dataset: Hope, British Columbia mountain block

This test set contains 16 adjacent 500 m COPC tiles from Natural Resources Canada's 2023 Hope, British Columbia collection. It covers a mountainous 2 x 2 km block rather than flat shoreline.

- Download size: **1,125,249,730 bytes** (about **1.05 GiB**)
- Source points: **131,931,025**
- Source point data at 16 bytes per `GpuPoint`: about **1.97 GiB**
- Coverage: a contiguous **2 x 2 km** block
- Elevation range: approximately **31 m to 433 m**
- Relief across the block: approximately **402 m**

That is intentionally much larger than the renderer's current 256 MiB point-memory budget. The renderer cannot keep the whole dataset resident, so flying across it exercises priority changes, loading, eviction, fence-safe retirement, hierarchy construction, and the draw budget.

These files use the COPC extension, but the current loader reads them sequentially as LAZ files. It does not yet use COPC's internal octree for HTTP range streaming. This test therefore stresses the system you have now; native COPC streaming can be a later milestone.

The Hope files use LAS point format 6, which has classification and intensity but no captured RGB. The renderer therefore starts in its improved terrain-style elevation mode:

- press **1** for source classification colours;
- press **2** for the elevation gradient.

The elevation gradient maps the full dataset height range through deep blue, teal, vegetation green, gold, rock, and pale peaks. It is generated by the shader from real elevation rather than stored photography.

Dataset sources:

- [NRCan LiDAR point-cloud product page](https://natural-resources.canada.ca/science-data/science-research/geomatics/new-lidar-point-clouds-product-canada-you-ve-never-seen)
- [NRCan CanElevation product specification](https://canelevation-lidar-point-clouds.s3.ca-central-1.amazonaws.com/pointclouds_nuagespoints/CanElevation-LiDARPointClouds_products_specs_EN.pdf)
- [Public Hope 2023 collection](https://canelevation-lidar-point-clouds.s3.ca-central-1.amazonaws.com/?list-type=2&prefix=pointclouds_nuagespoints%2FBC%2FHope_TB_2023%2F)

## Download command

Open PowerShell in the project root and paste this whole block. It creates a dedicated folder, downloads the 16 tiles, resumes partial files, and checks the final count and byte total.

```powershell
$tileDirectory = ".\test-tiles\hope-bc-mountains"
$baseUrl = "https://canelevation-lidar-point-clouds.s3.ca-central-1.amazonaws.com/pointclouds_nuagespoints/BC/Hope_TB_2023"
$fileNames = @(
    "Hope_014.copc.laz", "Hope_015.copc.laz",
    "Hope_016.copc.laz", "Hope_017.copc.laz",
    "Hope_028.copc.laz", "Hope_029.copc.laz",
    "Hope_030.copc.laz", "Hope_031.copc.laz",
    "Hope_044.copc.laz", "Hope_045.copc.laz",
    "Hope_046.copc.laz", "Hope_047.copc.laz",
    "Hope_061.copc.laz", "Hope_062.copc.laz",
    "Hope_063.copc.laz", "Hope_064.copc.laz"
)

New-Item -ItemType Directory -Force -Path $tileDirectory | Out-Null

foreach ($fileName in $fileNames) {
    $destination = Join-Path $tileDirectory $fileName
    $partial = "$destination.partial"
    $url = "$baseUrl/$fileName"

    if (Test-Path -LiteralPath $destination) {
        Write-Host "Already downloaded: $fileName"
        continue
    }

    Write-Host "Downloading: $fileName"
    curl.exe --fail --location --retry 4 --retry-delay 2 --continue-at - --output $partial $url

    if ($LASTEXITCODE -ne 0) {
        throw "Download failed: $fileName"
    }

    Move-Item -LiteralPath $partial -Destination $destination -Force
}

$downloadedTiles = @(Get-ChildItem -LiteralPath $tileDirectory -Filter "*.copc.laz" -File)
$downloadedBytes = ($downloadedTiles | Measure-Object -Property Length -Sum).Sum

if ($downloadedTiles.Count -ne 16 -or $downloadedBytes -ne 1125249730) {
    throw "Dataset check failed. Expected 16 files and 1125249730 bytes; got $($downloadedTiles.Count) files and $downloadedBytes bytes."
}

Write-Host "Dataset ready: $($downloadedTiles.Count) tiles, $downloadedBytes bytes."
```

If a transfer is interrupted, run the same block again. Existing completed files are skipped and `.partial` files resume.

## Build command

Run this from the project root in an ordinary PowerShell window:

```powershell
$build = 'call "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul && "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" --build "out\build\x64-Debug"'
& cmd.exe /d /s /c $build
```

## Start command

This passes every downloaded tile as a separate command-line argument. Avoid a backtick after the executable path.

```powershell
$tilePaths = @(
    Get-ChildItem -LiteralPath ".\test-tiles\hope-bc-mountains" -Filter "*.copc.laz" -File |
    Sort-Object Name |
    ForEach-Object FullName
)

if ($tilePaths.Count -ne 16) {
    throw "Expected 16 stress-test tiles; found $($tilePaths.Count)."
}

& ".\out\build\x64-Debug\PointCloudRenderer.exe" @tilePaths
```

To open the project in VS Code instead:

```powershell
code .
```

## Test sequence

1. Start with the camera framed over the entire 2 x 2 km block.
2. Let loading settle and record the on-screen counters.
3. Fly slowly from one corner to the opposite corner.
4. Turn around quickly several times to change tile priority.
5. Move close to the ground, then climb until the entire block is visible.
6. Toggle full detail and LOD modes only where the point budget makes the comparison meaningful.
7. Run the benchmark at the same fixed view before and after the Milestone 11 experiment.
8. Repeat the route while watching the D3D12 validation output.

Expected behavior with the current design:

- only the closest tiles that fit the 256 MiB budget become wanted;
- at most one wanted tile is decoded and uploaded per residency update;
- old tiles leave as priorities change;
- GPU resources wait for their fence before final destruction;
- selected point count stays within the global draw budget in LOD mode;
- the first visit to a tile may hitch because decoding and hierarchy construction are synchronous.

That last hitch is useful evidence. It identifies asynchronous loading and native COPC streaming as later work instead of hiding the limitation.

## Ready to move on when

- the paper can be explained in your own words from input points to final draw;
- the exact experiment and hypothesis are written down;
- the 16-tile route completes without crashes or D3D12 validation errors;
- residency visibly changes as the camera crosses the dataset;
- baseline timing and a short comparison recording have been captured;
- the result states both the visual improvement and its CPU/GPU cost.

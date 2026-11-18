#import <UIKit/UIKit.h>

#import "DiagViewController.h"

/* Round 29 ESP: one immutable draw snapshot, published with an atomic swap
   when the ESP pipeline completes. The overlay draws only from it. Scan
   activity writes the staging buffer in DiagViewController.h instead, so it
   can never blank the boxes while a scan is running. */
@interface DiagViewController (ESP)

/* ESP RESCAN button: shows the overlay and runs the normal pipeline. */
- (void)espRescan;

/* FOV calibration: publishes a copy of the current snapshot with the new
   fovX and redraws. Never reads the game. */
- (void)fovChanged:(UISegmentedControl *)sender;

/* Round 32: horizontal mirror toggle (persisted) and its button title. */
- (void)mirrorToggled:(UIButton *)sender;
- (void)espUpdateMirrorTitle;

/* 1 s age ticker and rotation redraw. */
- (void)espTick;
- (void)espRotated:(NSNotification *)note;

/* Worker thread, after the TRS camera decision: stages this scan's camera. */
- (void)espStageCamera:(const float *)camP haveP:(bool)haveCamP
                     f:(const float *)camF haveF:(bool)haveCamF;

/* Worker thread, after the pipeline: turns the staging buffer into a snapshot,
   publishes it, then prints "ESP done: ..." and the per-box file lines on the
   main thread. */
- (void)espCompleteScan;

/* STOP / EXIT: drop the snapshot and hide the overlay. */
- (void)espSnapshotClear;

@end

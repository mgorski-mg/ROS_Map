/*
 * Copyright (C) 2013 OSRF.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.example.myapplication2.app5;


import android.content.Context;
import android.os.Handler;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import android.view.ViewGroup;

import org.ros.android.view.RosImageView;
import org.ros.android.view.visualization.Camera;
import org.ros.android.view.visualization.RotateGestureDetector;
import org.ros.android.view.visualization.VisualizationView;
import org.ros.android.view.visualization.layer.CameraControlLayer;
import org.ros.android.view.visualization.layer.CameraControlListener;
import org.ros.concurrent.ListenerGroup;
import org.ros.concurrent.SignalRunnable;
import org.ros.node.ConnectedNode;
import org.ros.rosjava_geometry.FrameTransformTree;

import java.util.concurrent.ExecutorService;

/**
 * @author murase@jsk.imi.i.u-tokyo.ac.jp (Kazuto Murase)
 */
public class ViewControlLayer extends CameraControlLayer {

    private static final String ROBOT_FRAME = "base_footprint";
    private final Context context;
    private final ListenerGroup<CameraControlListener> listeners;

    private GestureDetector translateGestureDetector;
    private RotateGestureDetector rotateGestureDetector;
    private ScaleGestureDetector zoomGestureDetector;

    private RosImageView<sensor_msgs.CompressedImage> cameraView;
    private VisualizationView mapView;
    private ViewGroup mainLayout;
    private ViewGroup sideLayout;
    private boolean mapViewGestureAvaiable;


    private enum ViewMode {
        CAMERA, MAP
    }

    private ViewMode viewMode;


    public ViewControlLayer(Context context, ExecutorService executorService, RosImageView<sensor_msgs.CompressedImage> cameraView, VisualizationView mapView, ViewGroup mainLayout, ViewGroup sideLayout) {
        super(context, executorService);

        this.context = context;

        listeners = new ListenerGroup<CameraControlListener>(executorService);

        this.cameraView = cameraView;
        this.mapView = mapView;
        this.mainLayout = mainLayout;
        this.sideLayout = sideLayout;

        viewMode = ViewMode.CAMERA;
        this.cameraView.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                swapViews();
            }
        });

        this.mapView.setClickable(true);
        this.cameraView.setClickable(false);
        mapViewGestureAvaiable = false;
    }


    @Override
    public boolean onTouchEvent(VisualizationView view, MotionEvent event) {

        if (event.getAction() == MotionEvent.ACTION_UP) {
            mapViewGestureAvaiable = true;
        }
        if (viewMode == ViewMode.CAMERA) {
            swapViews();
            return true;
        } else {
            if (translateGestureDetector == null || rotateGestureDetector == null || zoomGestureDetector == null) {
                return false;
            }
            return translateGestureDetector.onTouchEvent(event) || rotateGestureDetector.onTouchEvent(event) || zoomGestureDetector.onTouchEvent(event);
        }
    }


    /**
     * Swap the camera and map views.
     */
    private void swapViews() {
        // Figure out where the views were...
        ViewGroup mapViewParent;
        ViewGroup cameraViewParent;

        if (viewMode == ViewMode.CAMERA) {

            mapViewParent = sideLayout;
            cameraViewParent = mainLayout;
        } else {

            mapViewParent = mainLayout;
            cameraViewParent = sideLayout;
        }
        int mapViewIndex = mapViewParent.indexOfChild(mapView);
        int cameraViewIndex = cameraViewParent.indexOfChild(cameraView);

        // Remove the views from their old locations...
        mapViewParent.removeView(mapView);
        cameraViewParent.removeView(cameraView);

        // Add them to their new location...
        mapViewParent.addView(cameraView, mapViewIndex);
        cameraViewParent.addView(mapView, cameraViewIndex);

        // Remeber that we are in the other mode now.
        if (viewMode == ViewMode.CAMERA) {
            viewMode = ViewMode.MAP;
            mapViewGestureAvaiable = false;
        } else {
            viewMode = ViewMode.CAMERA;
        }
        mapView.getCamera().jumpToFrame(ROBOT_FRAME);
        mapView.setClickable(viewMode != ViewMode.MAP);
        cameraView.setClickable(viewMode != ViewMode.CAMERA);

    }

    @Override
    public void onStart(ConnectedNode connectedNode, Handler handler, FrameTransformTree frameTransformTree, final Camera camera) {
        handler.post(new Runnable() {
            @Override
            public void run() {
                translateGestureDetector = new GestureDetector(context, new GestureDetector.SimpleOnGestureListener() {
                    @Override
                    public boolean onScroll(MotionEvent event1, MotionEvent event2, final float distanceX, final float distanceY) {
                        if (mapViewGestureAvaiable) {
                            camera.translate(-distanceX, distanceY);
                            listeners.signal(new SignalRunnable<CameraControlListener>() {
                                @Override
                                public void run(CameraControlListener listener) {
                                    listener.onTranslate(-distanceX, distanceY);
                                }
                            });
                            return true;
                        }

                        return false;
                    }
                });
                rotateGestureDetector = new RotateGestureDetector(new RotateGestureDetector.OnRotateGestureListener() {
                    @Override
                    public boolean onRotate(MotionEvent event1, MotionEvent event2, final double deltaAngle) {
                        if (mapViewGestureAvaiable) {
                            final double focusX = (event1.getX(0) + event1.getX(1)) / 2;
                            final double focusY = (event1.getY(0) + event1.getY(1)) / 2;
                            camera.rotate(focusX, focusY, deltaAngle);
                            listeners.signal(new SignalRunnable<CameraControlListener>() {
                                @Override
                                public void run(CameraControlListener listener) {
                                    listener.onRotate(focusX, focusY, deltaAngle);
                                }
                            });
                            // Don't consume this event in order to allow the zoom gesture
                            // to also be detected.
                            return false;
                        }

                        return true;
                    }
                });
                zoomGestureDetector = new ScaleGestureDetector(context, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
                    @Override
                    public boolean onScale(ScaleGestureDetector detector) {
                        if (!detector.isInProgress()) {
                            return false;
                        }
                        if (mapViewGestureAvaiable) {
                            final float focusX = detector.getFocusX();
                            final float focusY = detector.getFocusY();
                            final float factor = detector.getScaleFactor();
                            camera.zoom(focusX, focusY, factor);
                            listeners.signal(new SignalRunnable<CameraControlListener>() {
                                @Override
                                public void run(CameraControlListener listener) {
                                    listener.onZoom(focusX, focusY, factor);
                                }
                            });
                            return true;
                        }

                        return false;
                    }
                }
                );
            }
        });
    }
}
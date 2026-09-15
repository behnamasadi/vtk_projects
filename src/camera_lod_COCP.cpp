#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkFrustumSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLineSource.h>
#include <vtkMath.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPlanes.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

// -----------------------------------------------------------------------------
// Convert camera distance / projected object size into a conceptual COPC LOD.
//
// IMPORTANT:
// This is NOT a COPC specification rule.
// This is our viewer's LOD policy.
//
// Larger projected size -> deeper COPC level.
// -----------------------------------------------------------------------------
int SelectCopcLOD(double projectedPixels)
{
    if (projectedPixels < 30.0)
        return 0;

    if (projectedPixels < 60.0)
        return 1;

    if (projectedPixels < 120.0)
        return 2;

    if (projectedPixels < 240.0)
        return 3;

    if (projectedPixels < 480.0)
        return 4;

    if (projectedPixels < 960.0)
        return 5;

    return 6;
}

// -----------------------------------------------------------------------------
// Observer that reacts whenever the user moves the camera.
// -----------------------------------------------------------------------------
class CameraLODCallback : public vtkCommand
{
public:
    static CameraLODCallback *New()
    {
        return new CameraLODCallback;
    }

    vtkRenderer *Renderer = nullptr;
    vtkActor *FrustumActor = nullptr;
    vtkSphereSource *FocalPointSource = nullptr;
    vtkBillboardTextActor3D *FocalPointLabel = nullptr;
    vtkLineSource *CameraToFocalLine = nullptr;

    // Approximate world-space size of the object we are interested in.
    double ObjectWorldSize = 10.0;

    void Execute(vtkObject *, unsigned long eventId, void *) override
    {
        if (!Renderer)
            return;

        vtkCamera *camera = Renderer->GetActiveCamera();

        if (!camera)
            return;

        // ---------------------------------------------------------------------
        // Camera information
        // ---------------------------------------------------------------------

        double position[3];
        double focalPoint[3];

        camera->GetPosition(position);
        camera->GetFocalPoint(focalPoint);

        const double dx = focalPoint[0] - position[0];
        const double dy = focalPoint[1] - position[1];
        const double dz = focalPoint[2] - position[2];

        const double distance =
            std::sqrt(dx * dx + dy * dy + dz * dz);

        const double viewAngleDeg = camera->GetViewAngle();
        const double viewAngleRad =
            vtkMath::RadiansFromDegrees(viewAngleDeg);

        // ---------------------------------------------------------------------
        // Get viewport dimensions
        // ---------------------------------------------------------------------

        int *windowSize =
            Renderer->GetRenderWindow()->GetSize();

        const int width = std::max(windowSize[0], 1);
        const int height = std::max(windowSize[1], 1);

        const double aspect =
            static_cast<double>(width) /
            static_cast<double>(height);

        // ---------------------------------------------------------------------
        // Get camera frustum planes.
        //
        // Layout:
        //
        // left
        // right
        // bottom
        // top
        // near
        // far
        //
        // Each plane:
        //
        // A B C D
        //
        // Ax + By + Cz + D = 0
        // ---------------------------------------------------------------------

        double planes[24];

        camera->GetFrustumPlanes(
            aspect,
            planes);

        // ---------------------------------------------------------------------
        // Calculate projected size of our object.
        //
        // Approximation:
        //
        // pixels =
        //
        // objectWorldSize / distance
        //
        //       *
        //
        // viewportHeight /
        // (2 * tan(FOV / 2))
        // ---------------------------------------------------------------------

        double projectedPixels = 0.0;

        if (distance > 1e-9)
        {
            projectedPixels =
                (ObjectWorldSize / distance) *
                (static_cast<double>(height) /
                 (2.0 * std::tan(viewAngleRad * 0.5)));
        }

        const int lod =
            SelectCopcLOD(projectedPixels);
        // Keep the focal-point marker and view-direction line synchronized
        // with interactive camera changes.
        if (FocalPointSource)
        {
            FocalPointSource->SetCenter(focalPoint);
        }
        if (FocalPointLabel)
        {
            FocalPointLabel->SetPosition(focalPoint);
        }
        if (CameraToFocalLine)
        {
            CameraToFocalLine->SetPoint1(position);
            CameraToFocalLine->SetPoint2(focalPoint);
        }

        // ---------------------------------------------------------------------
        // Print everything.
        // ---------------------------------------------------------------------

        std::cout
            << "\n============================================\n";

        std::cout
            << "Camera event: "
            << eventId
            << "\n";

        std::cout
            << std::fixed
            << std::setprecision(3);

        std::cout
            << "Camera position : "
            << position[0] << ", "
            << position[1] << ", "
            << position[2] << "\n";

        std::cout
            << "Focal point     : "
            << focalPoint[0] << ", "
            << focalPoint[1] << ", "
            << focalPoint[2] << "\n";

        std::cout
            << "Distance        : "
            << distance
            << "\n";

        std::cout
            << "View angle      : "
            << viewAngleDeg
            << " deg\n";

        std::cout
            << "Viewport        : "
            << width
            << " x "
            << height
            << "\n";

        std::cout
            << "Projected size  : "
            << projectedPixels
            << " pixels\n";

        std::cout
            << "\nCOPC LOD DECISION\n";

        std::cout
            << "Selected level  : "
            << lod
            << "\n";

        std::cout
            << "\nFrustum planes:\n";

        const char *names[6] =
            {
                "LEFT  ",
                "RIGHT ",
                "BOTTOM",
                "TOP   ",
                "FAR   ",
                "NEAR  "};

        for (int i = 0; i < 6; ++i)
        {
            std::cout
                << names[i]
                << " : "
                << planes[i * 4 + 0] << " "
                << planes[i * 4 + 1] << " "
                << planes[i * 4 + 2] << " "
                << planes[i * 4 + 3]
                << "\n";
        }

        std::cout
            << "============================================\n";
    }
};

// -----------------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------------
int main(int, char *[])
{
    vtkNew<vtkNamedColors> colors;

    // =========================================================================
    // Renderer
    // =========================================================================

    vtkNew<vtkRenderer> renderer;

    renderer->SetBackground(
        0.08,
        0.10,
        0.15);

    // =========================================================================
    // Render window
    // =========================================================================

    vtkNew<vtkRenderWindow> renderWindow;

    renderWindow->AddRenderer(renderer);

    renderWindow->SetSize(
        1200,
        800);

    renderWindow->SetWindowName(
        "VTK Camera / Frustum / COPC LOD Demo");

    // =========================================================================
    // Interactor
    // =========================================================================

    vtkNew<vtkRenderWindowInteractor> interactor;

    interactor->SetRenderWindow(renderWindow);

    vtkNew<vtkInteractorStyleTrackballCamera> style;

    interactor->SetInteractorStyle(style);

    // =========================================================================
    // Create something resembling a point cloud
    // =========================================================================

    vtkNew<vtkPointSource> pointSource;

    pointSource->SetNumberOfPoints(10000);

    pointSource->SetCenter(
        0.0,
        0.0,
        0.0);

    pointSource->SetRadius(5.0);

    pointSource->Update();

    vtkNew<vtkPolyDataMapper> pointMapper;

    pointMapper->SetInputConnection(
        pointSource->GetOutputPort());

    vtkNew<vtkActor> pointActor;

    pointActor->SetMapper(pointMapper);

    pointActor->GetProperty()->SetColor(
        0.8,
        0.85,
        1.0);

    pointActor->GetProperty()->SetPointSize(2.0);
    pointActor->GetProperty()->SetOpacity(0.35);

    renderer->AddActor(pointActor);

    // =========================================================================
    // Add a cube so camera orientation is visually obvious
    // =========================================================================

    vtkNew<vtkCubeSource> cube;

    cube->SetCenter(
        0.0,
        0.0,
        0.0);

    cube->SetXLength(4.0);
    cube->SetYLength(4.0);
    cube->SetZLength(4.0);

    vtkNew<vtkPolyDataMapper> cubeMapper;

    cubeMapper->SetInputConnection(
        cube->GetOutputPort());

    vtkNew<vtkActor> cubeActor;

    cubeActor->SetMapper(cubeMapper);

    cubeActor->GetProperty()->SetRepresentationToWireframe();

    cubeActor->GetProperty()->SetColor(
        1.0,
        0.6,
        0.2);

    cubeActor->GetProperty()->SetLineWidth(2.0);

    renderer->AddActor(cubeActor);
    // World-coordinate reference at the scene origin.
    vtkNew<vtkAxesActor> worldAxes;
    worldAxes->SetTotalLength(3.0, 3.0, 3.0);
    worldAxes->SetShaftTypeToLine();
    renderer->AddActor(worldAxes);

    // =========================================================================
    // Camera
    // =========================================================================

    vtkCamera *camera =
        renderer->GetActiveCamera();

    camera->SetPosition(
        15.0,
        15.0,
        15.0);

    camera->SetFocalPoint(
        0.0,
        0.0,
        0.0);

    camera->SetViewUp(
        0.0,
        0.0,
        1.0);

    camera->SetViewAngle(30.0);

    // Use explicit clipping range so the frustum is easy to visualize.
    camera->SetClippingRange(
        1.0,
        50.0);

    // =========================================================================
    // Visualize focal point
    // =========================================================================

    vtkNew<vtkSphereSource> focalSphere;

    focalSphere->SetCenter(
        camera->GetFocalPoint());

    // This is the pivot around which vtkInteractorStyleTrackballCamera rotates.
    focalSphere->SetRadius(0.8);

    focalSphere->SetThetaResolution(24);
    focalSphere->SetPhiResolution(24);

    vtkNew<vtkPolyDataMapper> focalMapper;

    focalMapper->SetInputConnection(
        focalSphere->GetOutputPort());

    vtkNew<vtkActor> focalActor;

    focalActor->SetMapper(focalMapper);

    focalActor->GetProperty()->SetColor(
        1.0,
        0.0,
        0.0);

    renderer->AddActor(focalActor);

    vtkNew<vtkBillboardTextActor3D> focalLabel;
    focalLabel->SetInput("Camera pivot (focal point)");
    focalLabel->SetPosition(camera->GetFocalPoint());
    focalLabel->GetTextProperty()->SetColor(1.0, 0.2, 0.2);
    focalLabel->GetTextProperty()->SetFontSize(18);
    renderer->AddActor(focalLabel);
    // Visualize the camera's view direction from its position to the current
    // focal point. The callback below updates this as the camera moves.
    vtkNew<vtkLineSource> cameraToFocalLine;
    cameraToFocalLine->SetPoint1(camera->GetPosition());
    cameraToFocalLine->SetPoint2(camera->GetFocalPoint());
    vtkNew<vtkPolyDataMapper> cameraToFocalMapper;
    cameraToFocalMapper->SetInputConnection(cameraToFocalLine->GetOutputPort());
    vtkNew<vtkActor> cameraToFocalActor;
    cameraToFocalActor->SetMapper(cameraToFocalMapper);
    cameraToFocalActor->GetProperty()->SetColor(1.0, 1.0, 0.0);
    cameraToFocalActor->GetProperty()->SetLineWidth(3.0);
    renderer->AddActor(cameraToFocalActor);

    // =========================================================================
    // Camera frustum visualization
    // =========================================================================
    //
    // vtkFrustumSource converts vtkPlanes into visible geometry.
    // =========================================================================

    double aspect =
        static_cast<double>(renderWindow->GetSize()[0]) /
        static_cast<double>(renderWindow->GetSize()[1]);

    double frustumPlanesArray[24];

    camera->GetFrustumPlanes(
        aspect,
        frustumPlanesArray);

    vtkNew<vtkPlanes> frustumPlanes;

    frustumPlanes->SetFrustumPlanes(
        frustumPlanesArray);

    vtkNew<vtkFrustumSource> frustumSource;

    frustumSource->ShowLinesOn();

    frustumSource->SetPlanes(
        frustumPlanes);

    vtkNew<vtkPolyDataMapper> frustumMapper;

    frustumMapper->SetInputConnection(
        frustumSource->GetOutputPort());

    vtkNew<vtkActor> frustumActor;

    frustumActor->SetMapper(
        frustumMapper);

    frustumActor->GetProperty()->SetRepresentationToWireframe();

    frustumActor->GetProperty()->SetColor(
        0.0,
        1.0,
        0.0);

    frustumActor->GetProperty()->SetLineWidth(2.0);

    renderer->AddActor(frustumActor);

    // =========================================================================
    // Camera callback
    // =========================================================================

    vtkNew<CameraLODCallback> callback;

    callback->Renderer = renderer;
    callback->FrustumActor = frustumActor;
    callback->FocalPointSource = focalSphere;
    callback->FocalPointLabel = focalLabel;
    callback->CameraToFocalLine = cameraToFocalLine;

    callback->ObjectWorldSize = 10.0;

    // Camera ModifiedEvent happens during zoom/rotate/pan.
    camera->AddObserver(
        vtkCommand::ModifiedEvent,
        callback);

    // =========================================================================
    // Initial render
    // =========================================================================

    renderWindow->Render();

    callback->Execute(
        camera,
        vtkCommand::ModifiedEvent,
        nullptr);

    // =========================================================================
    // Start interaction
    // =========================================================================

    interactor->Initialize();
    interactor->Start();

    return EXIT_SUCCESS;
}
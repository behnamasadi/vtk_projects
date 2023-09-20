```cpp

vtkNew<vtkRenderWindowInteractor> interactor;

vtkNew<vtkTextActor> textActor;
textActor->SetInput("This is a test");


vtkNew<vtkTextWidget> textWidget;

vtkNew<vtkTextRepresentation> textRepresentation;
textRepresentation->GetPositionCoordinate()->SetValue(0.15, 0.15);
textRepresentation->GetPosition2Coordinate()->SetValue(0.7, 0.2);
textWidget->SetRepresentation(textRepresentation);

textWidget->SetInteractor(interactor);
textWidget->SetTextActor(textActor);
textWidget->SelectableOff();

textWidget->On();
```

[code](../src/TextWidget.cxx)

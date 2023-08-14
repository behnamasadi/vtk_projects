```cpp
coneActor->GetProperty()->SetColor(namedColor->GetColor3d("Gold").GetData());
coneActor->GetProperty()->SetAmbient(1.0);
coneActor->GetProperty()->SetDiffuse(0.0);
coneActor->GetProperty()->SetRepresentationToWireframe();
coneActor->GetProperty()->SetDiffuseColor(namedColor->GetColor3d("Red").GetData());
coneActor->GetProperty()->SetSpecular(0.3);
coneActor->GetProperty()->SetSpecularPower(30);
```

[code](../vtk/actor_properties.cpp)

Refs: [1](https://vtk.org/doc/nightly/html/classvtkProperty.html)

using System.Collections;
using System.Collections.Generic;

using UnityEngine;

using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.SceneSystem;
using UnityEngine.SceneManagement;
using System.Threading.Tasks;
using Microsoft.MixedReality.Toolkit.Input;

public class SceneController : MonoBehaviour
{
    private IMixedRealitySceneSystem sceneSystem;

    // Start is called before the first frame update
    void Start()
    {
        sceneSystem = MixedRealityToolkit.Instance.GetService<IMixedRealitySceneSystem>();
    }

    public void LoadScene(string sceneName)
    {
        PointerUtils.SetHandRayPointerBehavior(PointerBehavior.AlwaysOff);
        PointerUtils.SetGazePointerBehavior(PointerBehavior.AlwaysOff);
        PointerUtils.SetHandPokePointerBehavior(PointerBehavior.AlwaysOff);

        StartCoroutine(LoadSceneRoutine(sceneName));
    }

    private IEnumerator LoadSceneRoutine(string sceneName)
    {
        yield return LoadSceneRoutineAsync(sceneName);
    }

    private async Task LoadSceneRoutineAsync(string sceneName)
    {
        await sceneSystem.LoadContent(sceneName, LoadSceneMode.Single);
    }


}

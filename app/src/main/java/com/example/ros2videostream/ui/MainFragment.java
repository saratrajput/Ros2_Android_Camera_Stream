package com.example.ros2videostream.ui;

import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;

import com.example.ros2videostream.MainActivity;
import com.example.ros2videostream.R;
import com.example.ros2videostream.viewmodel.SettingViewModel;

public class MainFragment extends Fragment {
    private View view;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        if(view==null){
            view = inflater.inflate(R.layout.fragment_main, null);
            Log.d("view","load fragment_main");
        }
        //return super.onCreateView(inflater, container, savedInstanceState);
        return view;
    }

    @Override
    public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
        super.onViewCreated(view, savedInstanceState);
        Toolbar toolbar =view.findViewById(R.id.toolbar_main);
        ((AppCompatActivity) getActivity()).setSupportActionBar(toolbar);
        toolbar.setTitle(R.string.toolbar_title);
        setHasOptionsMenu(true);
        Button text = view.findViewById(R.id.text);
        text.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                ((MainActivity) requireActivity()).show_fragment_text();
            }
        });
        Button camera = view.findViewById(R.id.camera);
        camera.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ((MainActivity) requireActivity()).show_fragment_camerax();
            }
        });
        SettingViewModel settingViewModel = new ViewModelProvider(requireActivity()).get(SettingViewModel.class);
        Log.d("MgroupA", settingViewModel.getGroupA().getValue().getSettingcontent());
        Log.d("MgroupB", settingViewModel.getGroupB().getValue().getSettingcontent());
        Log.d("MgroupC", settingViewModel.getGroupC().getValue().getSettingcontent());
    }

    @Override
    public void onCreateOptionsMenu(Menu menu, MenuInflater inflater) {
        inflater.inflate(R.menu.toolbar_menu, menu);
    }

    @Override
    public boolean onOptionsItemSelected(@NonNull MenuItem item) {
        int menuItemId  = item.getItemId();
        if(menuItemId==R.id.menu1){
            ((MainActivity) requireActivity()).show_fragment_setting();
        }
        return super.onOptionsItemSelected(item);
    }
}

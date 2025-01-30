enum class Role { Main, Secondary };

static String getRoleStr(Role role) {
    String roleStr;
    switch (role) {
        case Role::Main:
            roleStr = "Main";
            break;
        case Role::Secondary:
            // Shorten word for AtomS3 small display
            roleStr = "Second";
            break;
        default:
            roleStr = "";
    }
    return roleStr;
}
